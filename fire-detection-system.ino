/* ---- Blynk template credentials ----
   These three lines MUST come before any #include, or the library will not
   compile. */
#define BLYNK_TEMPLATE_ID   "YOUR_BLYNK_TEMPLATE_ID"
#define BLYNK_TEMPLATE_NAME "Fire Detection System"
#define BLYNK_AUTH_TOKEN    "YOUR_BLYNK_AUTH_TOKEN"

#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <DHT.h>

/* ---------------- WI-FI CREDENTIALS ---------------- */
char ssid[] = "YOUR_WIFI_SSID";
char pass[] = "YOUR_WIFI_PASSWORD";
// Note: the ESP32 radio is 2.4 GHz only. A 5 GHz-only SSID will never connect.

/* ---------------- PIN ASSIGNMENTS ---------------- */
const int MQ2_PIN   = 34;   // MQ-2 analog out   (ADC1 - safe with Wi-Fi on)
const int SOUND_PIN = 35;   // LM393 analog out  (ADC1 - safe with Wi-Fi on)
const int DHT_PIN   = 27;   // DHT11 single-wire data

const int LED1 = 4;    // GREEN  (also the fault indicator - blinks on fault)
const int LED2 = 16;   // GREEN
const int LED3 = 17;   // YELLOW
const int LED4 = 18;   // YELLOW
const int LED5 = 19;   // RED
const int BUZZER = 25; // active piezo buzzer

const int leds[5] = {LED1, LED2, LED3, LED4, LED5};

/* ---------------- BLYNK VIRTUAL PINS ----------------
   Device -> App (telemetry)            App -> Device (control)
     V0 gas reading                       V6 buzzer mute switch
     V1 sound reading                     V7 lamp test button
     V2 temperature                       V8 simulated gas (test mode, 0 = off)
     V3 humidity
     V4 stage index (0-3)
     V5 system status text   (NEW) */
#define VP_GAS      V0
#define VP_SOUND    V1
#define VP_TEMP     V2
#define VP_HUM      V3
#define VP_STAGE    V4
#define VP_STATUS   V5
#define VP_MUTE     V6
#define VP_LAMPTEST V7
#define VP_SIMGAS   V8

/* ---------------- DHT CONFIGURATION ---------------- */
#define DHTTYPE DHT11
DHT dht(DHT_PIN, DHTTYPE);

/* ---------------- CALIBRATED THRESHOLDS (unchanged from midterm) -------- */
const int   GAS_WARN   = 700;    // Stage 1 - yellow caution tier engages
const int   GAS_ALERT  = 900;    // Stage 2 - red lamp engages
const int   GAS_ALARM  = 1000;   // Stage 3 - audible alarm
const float TEMP_ALARM = 34.0;   // Independent thermal trip, deg C

/* ---------------- ENHANCEMENT 01: DETECTION FILTER ---------------- */
const int   GAS_OVERSAMPLE  = 8;    // ADC reads averaged per sample
const int   CONFIRM_SAMPLES = 3;    // 3 x 400 ms = 1.2 s to escalate (Warning/Alert)
const int   CLEAR_SAMPLES   = 5;    // 5 x 400 ms = 2.0 s to step down
const int   GAS_HYST        = 50;   // step down only once gas < threshold - 50
const float TEMP_HYST       = 1.0;  // step down only once temp < 34.0 - 1.0

/* ---------------- ENHANCEMENT 02: FAULT DETECTION ---------------- */
const int           DHT_FAIL_LIMIT       = 5;      // 5 bad reads x 2.5 s = 12.5 s -> fault
const int           MQ2_RAIL_LOW         = 20;     // stuck near 0    -> signal lost
const int           MQ2_RAIL_HIGH        = 4090;   // stuck near 4095 -> short / saturated
const unsigned long MQ2_FAULT_MS         = 5000;   // must stay at a rail this long
const unsigned long FAULT_EVENT_COOLDOWN = 60000;  // min gap between repeat fault events
const unsigned long FAULT_BLINK_MS       = 500;    // LED1 blink half-period while faulted

/* ---------------- TEST MODE ---------------- */
const unsigned long SIM_TIMEOUT_MS = 300000;  // simulation switches itself off after 5 min

/* ---------------- TIMING ---------------- */
const unsigned long T_SENSE     = 400;    // gas + sound + stage evaluation
const unsigned long T_CLIMATE   = 2500;   // DHT11 needs >= 2 s between reads
const unsigned long T_PUSH      = 2000;   // telemetry to Blynk
const unsigned long T_HEALTH    = 1000;   // fault/link monitoring, status push
const unsigned long T_RECONNECT = 15000;  // network retry
const unsigned long BUZZ_ON     = 150;
const unsigned long BUZZ_OFF    = 400;
const unsigned long CRITICAL_LOCKOUT = 60000;  // see taskReconnect()

/* ---------------- STATE ---------------- */
enum Stage { STAGE_NORMAL = 0, STAGE_WARNING = 1, STAGE_ALERT = 2, STAGE_CRITICAL = 3 };
const char* STAGE_NAME[4] = { "NORMAL", "WARNING", "ALERT", "CRITICAL" };

Stage stage = STAGE_NORMAL;
unsigned long criticalSince = 0;   // millis() when Critical was entered, 0 if not

int   gasRaw   = 0;     // real MQ-2 reading (oversampled)
int   gasVal   = 0;     // value used for decisions (real, simulated or spike)
int   soundVal = 0;
float tempC    = NAN;
float humidity = NAN;

// Detection filter (Enhancement 01)
int   upCount   = 0;               // consecutive samples above the current stage
Stage upTarget  = STAGE_NORMAL;    // lowest stage seen during that run
int   upPeak    = 0;               // highest gas value seen during that run
int   downCount = 0;               // consecutive samples below the hysteresis band
Stage downTarget = STAGE_NORMAL;   // highest stage seen during that run
unsigned long filteredCount = 0;   // transients ignored since boot

// Faults (Enhancement 02)
const uint8_t FAULT_DHT = 0x01;
const uint8_t FAULT_MQ2 = 0x02;
uint8_t faults         = 0;        // active faults
uint8_t faultsNotified = 0;        // faults already reported to Blynk
int dhtFailStreak = 0;
unsigned long dhtFaultSince = 0;
unsigned long mq2RailSince  = 0;
unsigned long mq2FaultSince = 0;

// Connection monitoring (Enhancement 02)
struct LinkMonitor {
  const char*   name;
  bool          up;
  bool          everUp;
  unsigned long downSince;
  unsigned long attempts;      // reconnect attempts during the current outage
  unsigned long outages;       // outages since boot
  unsigned long lastOutageMs;  // duration of the most recent outage
};
LinkMonitor wifiLink  = { "Wi-Fi", false, false, 0, 0, 0, 0 };
LinkMonitor blynkLink = { "Blynk", false, false, 0, 0, 0, 0 };

// Test mode
bool simActive  = false;
int  simGas     = 0;
unsigned long simStarted = 0;
int  spikeValue = -1;              // >= 0: inject for exactly one sample

// Outputs
bool buzzerMuted = false;          // set from the Blynk app (V6)
bool buzzOn      = false;
unsigned long buzzMark = 0;
bool lampTestActive = false;
unsigned long lampTestStart = 0;
bool lampToneOn = false;
uint8_t lampMask = 0xFF;           // last LED pattern written, 0xFF = force refresh

BlynkTimer timer;

/* =========================================================================
   SMALL HELPERS
   ========================================================================= */
const char* faultSummary() {
  if ((faults & FAULT_DHT) && (faults & FAULT_MQ2)) return "DHT11+MQ-2";
  if (faults & FAULT_DHT) return "DHT11";
  if (faults & FAULT_MQ2) return "MQ-2";
  return "none";
}

String tempText() {
  return isnan(tempC) ? String("n/a") : String(tempC, 1) + "C";
}

int readGasOversampled() {
  long sum = 0;
  for (int i = 0; i < GAS_OVERSAMPLE; i++) sum += analogRead(MQ2_PIN);
  return (int)(sum / GAS_OVERSAMPLE);
}

/* =========================================================================
   OUTPUTS
   ========================================================================= */

/* LED pattern from the stage, written only when it changes.
   Bit 0 = LED1 (green) ... bit 4 = LED5 (red). */
void serviceLamps() {
  if (lampTestActive) return;

  uint8_t m;
  switch (stage) {
    case STAGE_CRITICAL:
    case STAGE_ALERT:   m = 0b11111; break;   // all five
    case STAGE_WARNING: m = 0b01111; break;   // 2 green + 2 yellow
    default:            m = 0b00011; break;   // green pair only
  }
  // Local fault indicator: LED1 blinks while any sensor fault is active.
  if (faults && ((millis() / FAULT_BLINK_MS) % 2)) m &= ~0b00001;

  if (m == lampMask) return;
  lampMask = m;
  for (int i = 0; i < 5; i++) digitalWrite(leds[i], (m >> i) & 1);
}

/* Non-blocking buzzer. Called from loop() every pass. */
void serviceBuzzer() {
  bool shouldSound = (stage == STAGE_CRITICAL) && !buzzerMuted;
  if (!shouldSound) {
    if (buzzOn) { noTone(BUZZER); buzzOn = false; }
    return;
  }
  unsigned long now = millis();
  if (buzzOn && (now - buzzMark >= BUZZ_ON)) {
    noTone(BUZZER); buzzOn = false; buzzMark = now;
  } else if (!buzzOn && (now - buzzMark >= BUZZ_OFF)) {
    tone(BUZZER, 2500); buzzOn = true; buzzMark = now;
  }
}

/* Non-blocking lamp test (the midterm version used delay() and froze
   sensing for ~1.2 s). Sweep LEDs on, hold, all off with a short beep. */
void startLampTest() {
  if (stage == STAGE_CRITICAL) {
    Serial.println("[LAMP] Refused - system is in CRITICAL");
    return;
  }
  lampTestActive = true;
  lampTestStart  = millis();
  lampToneOn     = false;
  Serial.println("[LAMP] Lamp test started");
}

void serviceLampTest() {
  if (!lampTestActive) return;
  unsigned long e = millis() - lampTestStart;
  if (stage == STAGE_CRITICAL) e = 0xFFFFFFFF;   // a real alarm aborts the test

  if (e < 750) {                                  // sweep: one more LED every 150 ms
    unsigned long n = e / 150 + 1;
    for (int i = 0; i < 5; i++) digitalWrite(leds[i], (unsigned long)i < n);
  } else if (e < 1050) {                          // hold all on
    for (int i = 0; i < 5; i++) digitalWrite(leds[i], HIGH);
  } else if (e < 1150) {                          // all off + beep
    for (int i = 0; i < 5; i++) digitalWrite(leds[i], LOW);
    if (!lampToneOn) { tone(BUZZER, 2000); lampToneOn = true; }
  } else {
    if (lampToneOn) { noTone(BUZZER); lampToneOn = false; }
    lampTestActive = false;
    lampMask = 0xFF;                              // force serviceLamps() to redraw
    Serial.println("[LAMP] Lamp test done");
  }
}

/* =========================================================================
   ENHANCEMENT 01 - HAZARD DETECTION
   ========================================================================= */

/* Instantaneous stage for one sample. With relaxed = true every threshold is
   lowered by its hysteresis margin; that is the level needed to STAY in a
   stage. A NaN temperature (DHT11 faulted) disables the thermal trip. */
Stage evaluateStage(int gas, float t, bool relaxed) {
  int   g = relaxed ? GAS_HYST  : 0;
  float h = relaxed ? TEMP_HYST : 0.0f;
  if (gas >= GAS_ALARM - g || (!isnan(t) && t >= TEMP_ALARM - h)) return STAGE_CRITICAL;
  if (gas >= GAS_ALERT - g) return STAGE_ALERT;
  if (gas >= GAS_WARN  - g) return STAGE_WARNING;
  return STAGE_NORMAL;
}

void reportFiltered() {
  filteredCount++;
  Serial.printf("[FILTER] Ignored brief gas reading: peak %d for %d sample%s "
                "(%d needed) - stage stays %s\n",
                upPeak, upCount, upCount == 1 ? "" : "s",
                CONFIRM_SAMPLES, STAGE_NAME[stage]);
}

void notifyHazard(Stage s) {
  if (!Blynk.connected()) return;
  String tag = simActive ? "[TEST] " : "";
  if (s == STAGE_CRITICAL) {
    Blynk.logEvent("critical_fire_alarm",
                   tag + "CRITICAL - gas " + gasVal + " / temp " + tempText());
  } else {
    Blynk.logEvent("elevated_gas_level",
                   tag + STAGE_NAME[s] + " - gas level " + gasVal);
  }
}

/* Decide the stage from the current sample plus recent history.
     - Critical:          immediate on a single sample.
     - Up to Warn/Alert:  CONFIRM_SAMPLES in a row above the current stage.
                          Promotes to the lowest level all of them agreed on.
     - Down:              CLEAR_SAMPLES in a row below the hysteresis band.
                          Drops to the highest level seen during that run. */
void updateStage() {
  Stage instant = evaluateStage(gasVal, tempC, false);
  Stage hold    = evaluateStage(gasVal, tempC, true);
  Stage prev    = stage;

  if (instant == STAGE_CRITICAL && stage != STAGE_CRITICAL) {
    stage = STAGE_CRITICAL;
    upCount = 0; downCount = 0;
    Serial.printf(">>> %s -> CRITICAL (immediate: gas=%d, temp=%s)\n",
                  STAGE_NAME[prev], gasVal, tempText().c_str());

  } else if (instant > stage) {
    downCount = 0;
    if (upCount == 0) { upTarget = instant; upPeak = gasVal; }
    else {
      if (instant < upTarget) upTarget = instant;
      if (gasVal > upPeak)    upPeak   = gasVal;
    }
    upCount++;
    if (upCount >= CONFIRM_SAMPLES) {
      stage = upTarget;
      Serial.printf(">>> %s -> %s (confirmed over %d samples, gas=%d)\n",
                    STAGE_NAME[prev], STAGE_NAME[stage], upCount, gasVal);
      upCount = 0;
    }

  } else if (hold < stage) {
    if (upCount > 0) reportFiltered();
    upCount = 0;
    if (downCount == 0 || hold > downTarget) downTarget = hold;
    downCount++;
    if (downCount >= CLEAR_SAMPLES) {
      stage = downTarget;
      Serial.printf("<<< %s -> %s (cleared after %d samples, gas=%d)\n",
                    STAGE_NAME[prev], STAGE_NAME[stage], downCount, gasVal);
      downCount = 0;
    }

  } else {
    if (upCount > 0) reportFiltered();
    upCount = 0;
    downCount = 0;
  }

  if (stage > prev) notifyHazard(stage);   // events on escalation only
}

/* =========================================================================
   ENHANCEMENT 02 - SENSOR FAULT DETECTION
   ========================================================================= */

/* A working MQ-2 never sits exactly at an ADC rail for seconds at a time.
   Near 0 usually means the signal wire is off; near 4095 means a short or a
   saturated sensor. Gas is still evaluated as read, so a saturated sensor
   keeps the system in Critical - the fault only adds a diagnostic. */
void checkGasSensor(int raw) {
  unsigned long now = millis();
  bool atRail = (raw <= MQ2_RAIL_LOW || raw >= MQ2_RAIL_HIGH);

  if (atRail) {
    if (mq2RailSince == 0) mq2RailSince = now;
    if (!(faults & FAULT_MQ2) && now - mq2RailSince >= MQ2_FAULT_MS) {
      faults |= FAULT_MQ2;
      mq2FaultSince = now;
      Serial.printf("[FAULT] MQ-2 reading stuck at %d for %lu s - %s\n",
                    raw, (now - mq2RailSince) / 1000,
                    raw >= MQ2_RAIL_HIGH ? "possible short or saturated sensor"
                                         : "check the AO wire on GPIO 34");
    }
  } else {
    mq2RailSince = 0;
    if (faults & FAULT_MQ2) {
      faults &= ~FAULT_MQ2;
      Serial.printf("[RECOVERED] MQ-2 reading back in range (%d) after %lu s\n",
                    raw, (now - mq2FaultSince) / 1000);
    }
  }
}

/* DHT11 read with fault tracking. A few failed reads are tolerated (the
   last good value is kept), but after DHT_FAIL_LIMIT in a row the sensor
   is declared faulty and its stale value is dropped. */
void taskClimate() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  bool valid = !isnan(t) && !isnan(h) && t > -10 && t < 80 && h >= 0 && h <= 100;

  if (valid) {
    tempC = t;
    humidity = h;
    dhtFailStreak = 0;
    if (faults & FAULT_DHT) {
      faults &= ~FAULT_DHT;
      Serial.printf("[RECOVERED] DHT11 responding again after %lu s (%.1fC, %.0f%%)\n",
                    (millis() - dhtFaultSince) / 1000, t, h);
    }
    return;
  }

  dhtFailStreak++;
  if (faults & FAULT_DHT) return;   // already reported

  if (dhtFailStreak < DHT_FAIL_LIMIT) {
    Serial.printf("[DIAG] DHT11 read failed (%d/%d) - keeping last good sample\n",
                  dhtFailStreak, DHT_FAIL_LIMIT);
  } else {
    faults |= FAULT_DHT;
    dhtFaultSince = millis();
    tempC = NAN;
    humidity = NAN;
    Serial.println("[FAULT] DHT11 not responding - check the data wire on GPIO 27.");
    Serial.println("        Temperature trip disabled; gas detection still active.");
  }
}

String faultMessage(uint8_t bit, unsigned long since) {
  String m;
  if (bit == FAULT_DHT) {
    m = "DHT11 not responding - temperature alarm disabled, gas detection still active";
  } else {
    m = "MQ-2 reading stuck at ";
    m += gasRaw;
    m += " - check sensor wiring";
  }
  unsigned long ago = (millis() - since) / 1000;
  if (ago >= 5) { m += " (detected "; m += ago; m += " s ago)"; }
  return m;
}

/* Report each new fault to Blynk once. A fault that happens while offline
   is reported as soon as the connection comes back. */
void sendFaultEvents() {
  faultsNotified &= faults;            // cleared faults can be reported again
  if (!Blynk.connected()) return;

  static unsigned long lastSent[2] = { 0, 0 };
  const uint8_t bits[2] = { FAULT_DHT, FAULT_MQ2 };

  for (int i = 0; i < 2; i++) {
    uint8_t b = bits[i];
    if (!(faults & b) || (faultsNotified & b)) continue;
    if (lastSent[i] != 0 && millis() - lastSent[i] < FAULT_EVENT_COOLDOWN) continue;

    String msg = faultMessage(b, b == FAULT_DHT ? dhtFaultSince : mq2FaultSince);
    Blynk.logEvent("sensor_fault", msg);
    faultsNotified |= b;
    lastSent[i] = millis();
    Serial.printf("[BLYNK] Fault event sent: %s\n", msg.c_str());
  }
}

/* =========================================================================
   ENHANCEMENT 02 - CONNECTION MONITORING
   ========================================================================= */

enum LinkChange { LINK_NO_CHANGE, LINK_LOST, LINK_FIRST_UP, LINK_RESTORED };

LinkChange monitorLink(LinkMonitor &l, bool nowUp) {
  if (nowUp == l.up) return LINK_NO_CHANGE;
  unsigned long now = millis();
  l.up = nowUp;

  if (!nowUp) {
    l.downSince = now;
    l.attempts  = 0;
    Serial.printf("[NET] %s connection lost\n", l.name);
    return LINK_LOST;
  }
  if (!l.everUp) {
    l.everUp = true;
    Serial.printf("[NET] %s connected\n", l.name);
    return LINK_FIRST_UP;
  }
  l.outages++;
  l.lastOutageMs = now - l.downSince;
  Serial.printf("[NET] %s restored after %lu s (%lu reconnect attempts, outage #%lu)\n",
                l.name, l.lastOutageMs / 1000, l.attempts, l.outages);
  return LINK_RESTORED;
}

/* True while a fresh Critical alarm should not be interrupted by a blocking
   Blynk.connect(). Expires after CRITICAL_LOCKOUT so a warm room cannot keep
   the node offline forever. */
bool alarmHasPriority() {
  return stage == STAGE_CRITICAL && criticalSince != 0 &&
         (millis() - criticalSince) < CRITICAL_LOCKOUT;
}

void tryBlynkConnect() {
  blynkLink.attempts++;
  Serial.printf("[NET] Blynk down, reconnect attempt %lu...\n", blynkLink.attempts);
  Blynk.connect(1500);
}

String buildStatus() {
  String s;
  if (faults & FAULT_DHT) s = "DHT11 FAULT";
  if (faults & FAULT_MQ2) { if (s.length()) s += " | "; s += "MQ-2 FAULT"; }
  if (!s.length()) s = "Sensors OK";
  if (simActive) s = String("TEST MODE | ") + s;
  if (blynkLink.outages) {
    s += " | last outage ";
    s += blynkLink.lastOutageMs / 1000;
    s += " s";
  }
  return s;
}

void pushStatus(bool force) {
  if (!Blynk.connected()) return;
  static String last;
  String s = buildStatus();
  if (!force && s == last) return;
  Blynk.virtualWrite(VP_STATUS, s);
  last = s;
}

/* =========================================================================
   TEST MODE (safe demonstration without flame or gas)
   ========================================================================= */
void stopSimulation(const char* why) {
  if (!simActive) return;
  simActive = false;
  Serial.printf("[TEST] Simulation off (%s) - using real MQ-2 readings\n", why);
  if (Blynk.connected()) Blynk.virtualWrite(VP_SIMGAS, 0);
}

void startSimulation(int v, const char* src) {
  if (v <= 0) { stopSimulation(src); return; }
  simGas     = constrain(v, 1, 4095);
  simActive  = true;
  simStarted = millis();             // each new value restarts the timeout
  Serial.printf("[TEST] Simulated gas = %d (from %s), auto-off in %lu s\n",
                simGas, src, SIM_TIMEOUT_MS / 1000);
}

void printHealthReport() {
  unsigned long now = millis();
  Serial.println("\n========== SYSTEM HEALTH ==========");
  Serial.printf("Uptime        : %lu s\n", now / 1000);
  Serial.printf("Stage         : %s\n", STAGE_NAME[stage]);
  Serial.printf("MQ-2          : raw %d, used %d  [%s]\n", gasRaw, gasVal,
                (faults & FAULT_MQ2) ? "FAULT" : "OK");
  Serial.printf("DHT11         : %s / %.0f%%  [%s, fail streak %d]\n",
                tempText().c_str(), humidity,
                (faults & FAULT_DHT) ? "FAULT" : "OK", dhtFailStreak);
  Serial.printf("Filtered      : %lu brief gas readings ignored\n", filteredCount);
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("Wi-Fi         : up, %s, RSSI %d dBm\n",
                  WiFi.localIP().toString().c_str(), (int)WiFi.RSSI());
  } else {
    Serial.printf("Wi-Fi         : DOWN for %lu s\n",
                  wifiLink.downSince ? (now - wifiLink.downSince) / 1000 : now / 1000);
  }
  Serial.printf("Blynk         : %s, %lu outage(s)", Blynk.connected() ? "up" : "DOWN",
                blynkLink.outages);
  if (blynkLink.outages) Serial.printf(", last %lu s", blynkLink.lastOutageMs / 1000);
  Serial.println();
  Serial.printf("Buzzer        : %s\n", buzzerMuted ? "MUTED" : "armed");
  if (simActive) {
    Serial.printf("Test mode     : ON, gas %d, %lu s left\n", simGas,
                  (SIM_TIMEOUT_MS - (now - simStarted)) / 1000);
  } else {
    Serial.println("Test mode     : off");
  }
  Serial.println("===================================\n");
}

void printHelp() {
  Serial.println("\nCommands:");
  Serial.println("  status       - system health report");
  Serial.println("  sim <value>  - simulate a steady gas reading (e.g. sim 950)");
  Serial.println("  sim off      - back to the real MQ-2");
  Serial.println("  spike <val>  - inject ONE sample (shows the transient filter)");
  Serial.println("  lamptest     - run the LED/buzzer lamp test");
  Serial.println("  help         - this list\n");
}

void handleCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();
  if (cmd == "help") {
    printHelp();
  } else if (cmd == "status") {
    printHealthReport();
  } else if (cmd.startsWith("sim")) {
    String arg = cmd.substring(3);
    arg.trim();
    if (arg == "off" || arg.length() == 0) stopSimulation("stopped from Serial");
    else startSimulation(arg.toInt(), "Serial");
  } else if (cmd.startsWith("spike")) {
    String arg = cmd.substring(5);
    arg.trim();
    spikeValue = constrain((int)arg.toInt(), 0, 4095);
    Serial.printf("[TEST] Next sample will read %d (one sample only)\n", spikeValue);
  } else if (cmd == "lamptest") {
    startLampTest();
  } else if (cmd.length()) {
    Serial.println("Unknown command - type 'help'");
  }
}

void serviceSerial() {
  static String buf;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (buf.length()) handleCommand(buf);
      buf = "";
    } else if (buf.length() < 40) {
      buf += c;
    }
  }
}

/* =========================================================================
   PERIODIC TASKS
   ========================================================================= */
void taskSense() {
  gasRaw   = readGasOversampled();
  soundVal = analogRead(SOUND_PIN);
  checkGasSensor(gasRaw);

  if (spikeValue >= 0) {
    gasVal = spikeValue;
    spikeValue = -1;
    Serial.printf("[TEST] Injected one-sample reading: %d\n", gasVal);
  } else if (simActive) {
    gasVal = simGas;
  } else {
    gasVal = gasRaw;
  }

  updateStage();

  // Track how long we have been Critical (see taskReconnect).
  if (stage == STAGE_CRITICAL) {
    if (criticalSince == 0) criticalSince = millis();
  } else {
    criticalSince = 0;
  }

  char pending[24] = "";
  if (upCount > 0) {
    snprintf(pending, sizeof(pending), " (->%s %d/%d)",
             STAGE_NAME[upTarget], upCount, CONFIRM_SAMPLES);
  } else if (downCount > 0) {
    snprintf(pending, sizeof(pending), " (clear %d/%d)", downCount, CLEAR_SAMPLES);
  }

  Serial.printf("Gas:%4d%s | Sound:%4d | Temp:%5.1fC | Hum:%5.1f%% | Stage:%s%s | "
                "Faults:%s | WiFi:%s Blynk:%s\n",
                gasVal, simActive ? "(sim)" : "", soundVal, tempC, humidity,
                STAGE_NAME[stage], pending, faultSummary(),
                WiFi.status() == WL_CONNECTED ? "up" : "down",
                Blynk.connected() ? "up" : "down");
}

/* Send-on-change telemetry (free-tier message budget), with a full resend
   every 60 s and on every (re)connect. */
void pushTelemetry(bool force) {
  if (!Blynk.connected()) return;

  static int   pGas = -1, pSound = -1, pStage = -1;
  static float pTemp = -999, pHum = -999;
  static unsigned long lastFull = 0;

  bool full = force || (millis() - lastFull > 60000);
  if (full) lastFull = millis();

  if (full || gasVal   != pGas)     { Blynk.virtualWrite(VP_GAS,   gasVal);      pGas   = gasVal; }
  if (full || soundVal != pSound)   { Blynk.virtualWrite(VP_SOUND, soundVal);    pSound = soundVal; }
  if (full || (int)stage != pStage) { Blynk.virtualWrite(VP_STAGE, (int)stage); pStage = (int)stage; }
  if (!isnan(tempC)    && (full || tempC    != pTemp)) { Blynk.virtualWrite(VP_TEMP, tempC);    pTemp = tempC; }
  if (!isnan(humidity) && (full || humidity != pHum))  { Blynk.virtualWrite(VP_HUM,  humidity); pHum  = humidity; }
  pushStatus(full);
}

void taskPush() { pushTelemetry(false); }

void taskHealth() {
  LinkChange wifiChange = monitorLink(wifiLink, WiFi.status() == WL_CONNECTED);

  // Wi-Fi just came up: reconnect Blynk now instead of waiting for the
  // next 15 s reconnect tick.
  bool wifiCameUp = (wifiChange == LINK_RESTORED || wifiChange == LINK_FIRST_UP);
  if (wifiCameUp && !Blynk.connected() && !alarmHasPriority()) tryBlynkConnect();

  LinkChange blynkChange = monitorLink(blynkLink, Blynk.connected());

  if (blynkChange == LINK_RESTORED && Blynk.connected()) {
    String msg = "Back online after ";
    msg += blynkLink.lastOutageMs / 1000;
    msg += " s offline (";
    msg += blynkLink.attempts;
    msg += " reconnect attempts). Sensors: ";
    msg += faults ? faultSummary() : "OK";
    if (faults) msg += " FAULT";
    Blynk.logEvent("connection_restored", msg);
    Serial.printf("[BLYNK] %s\n", msg.c_str());
  }

  if (simActive && millis() - simStarted >= SIM_TIMEOUT_MS) {
    stopSimulation("timed out");
  }

  sendFaultEvents();
  pushStatus(false);
}

void taskReconnect() {
  /* The alarm normally takes priority over the link, because Blynk.connect()
     blocks for its timeout. The lockout expires after 60 s of continuous
     Critical so a warm room cannot keep the node offline forever. */
  if (alarmHasPriority()) {
    if (WiFi.status() != WL_CONNECTED || !Blynk.connected()) {
      Serial.println("[NET] Reconnect deferred - CRITICAL alarm has priority");
    }
    return;
  }

  if (WiFi.status() != WL_CONNECTED) {
    wifiLink.attempts++;
    blynkLink.attempts++;
    Serial.printf("[NET] Wi-Fi down for %lu s, reconnect attempt %lu...\n",
                  wifiLink.downSince ? (millis() - wifiLink.downSince) / 1000 : millis() / 1000,
                  wifiLink.attempts);
    WiFi.begin(ssid, pass);
    return;
  }
  if (!Blynk.connected()) tryBlynkConnect();
}

/* =========================================================================
   BLYNK APP -> DEVICE HANDLERS
   ========================================================================= */
BLYNK_WRITE(VP_MUTE) {                 // switch widget
  buzzerMuted = (param.asInt() == 1);
  Serial.printf("Buzzer %s from app\n", buzzerMuted ? "MUTED" : "ARMED");
  if (buzzerMuted && buzzOn) { noTone(BUZZER); buzzOn = false; }
}

BLYNK_WRITE(VP_LAMPTEST) {             // button widget (Push mode)
  if (param.asInt() == 1) startLampTest();
}

BLYNK_WRITE(VP_SIMGAS) {               // slider 0-4095, 0 = test mode off
  startSimulation(param.asInt(), "Blynk");
}

BLYNK_CONNECTED() {
  Serial.println("Blynk connected");
  Blynk.syncVirtual(VP_MUTE);          // restore the mute state after a reboot
  if (!simActive) Blynk.virtualWrite(VP_SIMGAS, 0);   // never resume a stale test
  pushTelemetry(true);                 // full snapshot, including status text
}

/* =========================================================================
   SETUP
   ========================================================================= */
void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(MQ2_PIN, INPUT);
  pinMode(SOUND_PIN, INPUT);
  pinMode(BUZZER, OUTPUT);
  for (int i = 0; i < 5; i++) pinMode(leds[i], OUTPUT);
  dht.begin();

  Serial.println("\n--- Enhanced Fire Detection System Initializing ---");

  // Output self-test (blocking is fine here - nothing is being monitored yet)
  for (int i = 0; i < 5; i++) { digitalWrite(leds[i], HIGH); delay(150); }
  delay(300);
  for (int i = 0; i < 5; i++) digitalWrite(leds[i], LOW);
  tone(BUZZER, 2000); delay(100); noTone(BUZZER);

  // Network: configure, then attempt once. Failure is not fatal.
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  Serial.print("Connecting to Wi-Fi");
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
    delay(300); Serial.print(".");
  }
  Serial.println(WiFi.status() == WL_CONNECTED ? " connected" : " timed out");

  Blynk.config(BLYNK_AUTH_TOKEN);
  if (WiFi.status() == WL_CONNECTED) Blynk.connect(3000);

  // Sensor self-check: first readings, so faults show up from the start.
  taskClimate();
  gasRaw = readGasOversampled();
  gasVal = gasRaw;
  Serial.printf("[SELFTEST] MQ-2: %d %s\n", gasRaw,
                (gasRaw <= MQ2_RAIL_LOW || gasRaw >= MQ2_RAIL_HIGH)
                  ? "(at ADC rail - check wiring)" : "(OK)");
  Serial.printf("[SELFTEST] DHT11: %s\n",
                isnan(tempC) ? "no reading yet" : (tempText() + " (OK)").c_str());

  timer.setInterval(T_SENSE,     taskSense);
  timer.setInterval(T_CLIMATE,   taskClimate);
  timer.setInterval(T_PUSH,      taskPush);
  timer.setInterval(T_HEALTH,    taskHealth);
  timer.setInterval(T_RECONNECT, taskReconnect);

  Serial.println("--- Fire & Safety Monitoring Active ---");
  Serial.println("Type 'help' for test and diagnostic commands.");
}

/* =========================================================================
   LOOP - no delay() anywhere below this line
   ========================================================================= */
void loop() {
  if (Blynk.connected()) Blynk.run();
  timer.run();
  serviceSerial();
  serviceLampTest();
  serviceLamps();
  serviceBuzzer();
}
