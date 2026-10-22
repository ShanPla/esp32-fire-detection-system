/* =========================================================================
   Enhanced ESP32-Based Fire Detection System
   with Improved Hazard Detection and System Health Monitoring

   IoT Final Project - BSCS C4A
   Platon, Shan Christian M. | Resurreccion, Steephen L. | Escobin, Maria Fae

   Enhancements over the midterm prototype (same hardware):
   01 Hazard detection - Warning/Alert must be confirmed over several
      samples, Critical still trips immediately, hysteresis on the way down.
   02 System health - DHT11 / MQ-2 fault detection and Wi-Fi / Blynk outage
      monitoring, reported on the Serial Monitor and the Blynk dashboard.
   ========================================================================= */

/* Blynk credentials - must come before any #include */
#define BLYNK_TEMPLATE_ID   "YOUR_BLYNK_TEMPLATE_ID"
#define BLYNK_TEMPLATE_NAME "Fire Detection System"
#define BLYNK_AUTH_TOKEN    "YOUR_BLYNK_AUTH_TOKEN"
#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <DHT.h>

char ssid[] = "YOUR_WIFI_SSID";       // 2.4 GHz networks only
char pass[] = "YOUR_WIFI_PASSWORD";

/* ---------------- PINS ---------------- */
const int MQ2_PIN   = 34;   // MQ-2 analog out
const int SOUND_PIN = 35;   // LM393 analog out
const int DHT_PIN   = 27;   // DHT11 data
const int BUZZER    = 25;
const int LED1 = 4, LED2 = 16, LED3 = 17, LED4 = 18, LED5 = 19;  // G G Y Y R
const int leds[5] = {LED1, LED2, LED3, LED4, LED5};

/* ---------------- BLYNK VIRTUAL PINS ---------------- */
#define VP_GAS      V0   // gas reading
#define VP_SOUND    V1   // sound reading
#define VP_TEMP     V2   // temperature
#define VP_HUM      V3   // humidity
#define VP_STAGE    V4   // stage 0-3
#define VP_STATUS   V5   // NEW: system status text
#define VP_MUTE     V6   // buzzer mute switch
#define VP_LAMPTEST V7   // lamp test button
#define VP_SIMGAS   V8   // NEW: test gas slider (0 = off)

DHT dht(DHT_PIN, DHT11);

/* ---------------- THRESHOLDS (same as midterm) ---------------- */
const int   GAS_WARN   = 700;
const int   GAS_ALERT  = 900;
const int   GAS_ALARM  = 1000;
const float TEMP_ALARM = 34.0;

/* ---------------- ENHANCEMENT 01 SETTINGS ---------------- */
const int   CONFIRM_SAMPLES = 3;    // 3 x 400 ms = 1.2 s to escalate to Warning/Alert
const int   CLEAR_SAMPLES   = 5;    // 5 x 400 ms = 2.0 s to step down
const int   GAS_HYST        = 50;   // gas must drop 50 below a threshold to clear it
const float TEMP_HYST       = 1.0;  // temp must drop 1 C below 34 C to clear it

/* ---------------- ENHANCEMENT 02 SETTINGS ---------------- */
const int           DHT_FAIL_LIMIT = 5;      // 5 failed reads (~12.5 s) -> DHT11 fault
const int           MQ2_RAIL_LOW   = 20;     // MQ-2 stuck near 0    -> wire off
const int           MQ2_RAIL_HIGH  = 4090;   // MQ-2 stuck near 4095 -> short / saturated
const unsigned long MQ2_FAULT_MS   = 5000;   // how long it must stay stuck

/* ---------------- TIMING ---------------- */
const unsigned long T_SENSE = 400, T_CLIMATE = 2500, T_PUSH = 2000;
const unsigned long T_HEALTH = 1000, T_RECONNECT = 15000;
const unsigned long BUZZ_ON = 150, BUZZ_OFF = 400;
const unsigned long CRITICAL_LOCKOUT = 60000;   // pause reconnects during a new alarm
const unsigned long SIM_TIMEOUT_MS   = 300000;  // test mode switches off after 5 min

/* ---------------- STATE ---------------- */
enum Stage { STAGE_NORMAL, STAGE_WARNING, STAGE_ALERT, STAGE_CRITICAL };
const char* STAGE_NAME[4] = { "NORMAL", "WARNING", "ALERT", "CRITICAL" };
Stage stage = STAGE_NORMAL;
unsigned long criticalSince = 0;

int   gasRaw = 0, gasVal = 0, soundVal = 0;   // gasVal = value used for decisions
float tempC = NAN, humidity = NAN;

// Enhancement 01 - detection filter
int   upCount = 0, downCount = 0, upPeak = 0;
Stage upTarget = STAGE_NORMAL, downTarget = STAGE_NORMAL;

// Enhancement 02 - faults
const uint8_t FAULT_DHT = 0x01, FAULT_MQ2 = 0x02;
uint8_t faults = 0, faultsNotified = 0;
int dhtFailStreak = 0;
unsigned long dhtFaultSince = 0, mq2RailSince = 0;

// Enhancement 02 - connection monitoring
struct LinkMonitor {
  const char* name;
  bool up, everUp;
  unsigned long downSince, attempts, outages, lastOutageMs;
};
LinkMonitor wifiLink  = { "Wi-Fi", false, false, 0, 0, 0, 0 };
LinkMonitor blynkLink = { "Blynk", false, false, 0, 0, 0, 0 };

// Test mode
bool simActive = false;
int  simGas = 0, spikeValue = -1;
unsigned long simStarted = 0;

// Buzzer
bool buzzerMuted = false, buzzOn = false;
unsigned long buzzMark = 0;

BlynkTimer timer;

/* =========================================================================
   OUTPUTS (same as midterm)
   ========================================================================= */
void setLamps(bool a, bool b, bool c, bool d, bool e) {
  digitalWrite(LED1, a); digitalWrite(LED2, b); digitalWrite(LED3, c);
  digitalWrite(LED4, d); digitalWrite(LED5, e);
}

void applyStageOutputs() {
  switch (stage) {
    case STAGE_CRITICAL:
    case STAGE_ALERT:   setLamps(1,1,1,1,1); break;   // all five
    case STAGE_WARNING: setLamps(1,1,1,1,0); break;   // 2 green + 2 yellow
    default:            setLamps(1,1,0,0,0); break;   // 2 green
  }
}

void serviceBuzzer() {   // non-blocking pulse while Critical
  bool shouldSound = (stage == STAGE_CRITICAL) && !buzzerMuted;
  if (!shouldSound) {
    if (buzzOn) { noTone(BUZZER); buzzOn = false; }
    return;
  }
  unsigned long now = millis();
  if (buzzOn && now - buzzMark >= BUZZ_ON)       { noTone(BUZZER);       buzzOn = false; buzzMark = now; }
  else if (!buzzOn && now - buzzMark >= BUZZ_OFF) { tone(BUZZER, 2500);  buzzOn = true;  buzzMark = now; }
}

/* =========================================================================
   ENHANCEMENT 01 - HAZARD DETECTION
   ========================================================================= */

// Average 8 ADC reads per sample to reduce noise.
int readGas() {
  long sum = 0;
  for (int i = 0; i < 8; i++) sum += analogRead(MQ2_PIN);
  return sum / 8;
}

/* Stage for a single sample. relaxed = true lowers every threshold by its
   hysteresis margin - that is the level needed to STAY in a stage.
   A NaN temperature (DHT11 fault) disables the thermal trip. */
Stage evaluateStage(int gas, float t, bool relaxed) {
  int   g = relaxed ? GAS_HYST  : 0;
  float h = relaxed ? TEMP_HYST : 0;
  if (gas >= GAS_ALARM - g || (!isnan(t) && t >= TEMP_ALARM - h)) return STAGE_CRITICAL;
  if (gas >= GAS_ALERT - g) return STAGE_ALERT;
  if (gas >= GAS_WARN  - g) return STAGE_WARNING;
  return STAGE_NORMAL;
}

void notifyHazard() {   // Blynk event on escalation
  if (!Blynk.connected()) return;
  String tag = simActive ? "[TEST] " : "";
  if (stage == STAGE_CRITICAL)
    Blynk.logEvent("critical_fire_alarm", tag + "CRITICAL - gas " + gasVal);
  else
    Blynk.logEvent("elevated_gas_level", tag + STAGE_NAME[stage] + " - gas " + gasVal);
}

/* Critical:      immediate on one sample.
   Warning/Alert: needs CONFIRM_SAMPLES in a row above the current stage.
   Stepping down: needs CLEAR_SAMPLES in a row below the hysteresis band. */
void updateStage() {
  Stage instant = evaluateStage(gasVal, tempC, false);
  Stage hold    = evaluateStage(gasVal, tempC, true);
  Stage prev    = stage;

  if (instant == STAGE_CRITICAL && stage != STAGE_CRITICAL) {
    stage = STAGE_CRITICAL;
    upCount = downCount = 0;

  } else if (instant > stage) {             // possible escalation - count it
    downCount = 0;
    if (upCount == 0) { upTarget = instant; upPeak = gasVal; }
    upTarget = min(upTarget, instant);      // only go as high as every sample agreed
    upPeak   = max(upPeak, gasVal);
    if (++upCount >= CONFIRM_SAMPLES) { stage = upTarget; upCount = 0; }

  } else {
    if (upCount > 0) {                      // run ended before confirmation
      Serial.printf("[FILTER] Ignored brief gas reading: peak %d for %d sample(s)\n",
                    upPeak, upCount);
      upCount = 0;
    }
    if (hold < stage) {                     // possible step down - count it
      if (downCount == 0 || hold > downTarget) downTarget = hold;
      if (++downCount >= CLEAR_SAMPLES) { stage = downTarget; downCount = 0; }
    } else {
      downCount = 0;
    }
  }

  if (stage != prev) {
    Serial.printf("%s %s -> %s (gas=%d, temp=%.1fC)\n", stage > prev ? ">>>" : "<<<",
                  STAGE_NAME[prev], STAGE_NAME[stage], gasVal, tempC);
    if (stage > prev) notifyHazard();
  }
}

/* =========================================================================
   ENHANCEMENT 02 - SENSOR FAULT DETECTION
   ========================================================================= */

// A working MQ-2 never sits at 0 or 4095 for seconds at a time.
void checkGasSensor(int raw) {
  bool stuck = (raw <= MQ2_RAIL_LOW || raw >= MQ2_RAIL_HIGH);
  if (!stuck) {
    mq2RailSince = 0;
    if (faults & FAULT_MQ2) { faults &= ~FAULT_MQ2; Serial.println("[RECOVERED] MQ-2 back in range"); }
    return;
  }
  if (mq2RailSince == 0) mq2RailSince = millis();
  if (!(faults & FAULT_MQ2) && millis() - mq2RailSince >= MQ2_FAULT_MS) {
    faults |= FAULT_MQ2;
    Serial.printf("[FAULT] MQ-2 stuck at %d - check wiring on GPIO 34\n", raw);
  }
}

// DHT11: a few failed reads are tolerated, then the sensor is marked faulty
// and its old value is dropped instead of being trusted forever.
void taskClimate() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();

  if (!isnan(t) && !isnan(h)) {
    tempC = t; humidity = h; dhtFailStreak = 0;
    if (faults & FAULT_DHT) { faults &= ~FAULT_DHT; Serial.println("[RECOVERED] DHT11 responding again"); }
    return;
  }

  dhtFailStreak++;
  if (faults & FAULT_DHT) return;
  if (dhtFailStreak < DHT_FAIL_LIMIT) {
    Serial.printf("[DIAG] DHT11 read failed (%d/%d)\n", dhtFailStreak, DHT_FAIL_LIMIT);
  } else {
    faults |= FAULT_DHT;
    dhtFaultSince = millis();
    tempC = humidity = NAN;
    Serial.println("[FAULT] DHT11 not responding - temperature trip disabled, gas still active");
  }
}

// Send each new fault to Blynk once. Faults that happen offline are sent on reconnect.
void sendFaultEvents() {
  faultsNotified &= faults;               // a cleared fault can be reported again
  if (!Blynk.connected()) return;

  if ((faults & FAULT_DHT) && !(faultsNotified & FAULT_DHT)) {
    String msg = "DHT11 not responding - temperature alarm disabled, gas detection still active";
    unsigned long ago = (millis() - dhtFaultSince) / 1000;
    if (ago >= 5) { msg += " (detected "; msg += ago; msg += " s ago)"; }
    Blynk.logEvent("sensor_fault", msg);
    faultsNotified |= FAULT_DHT;
  }
  if ((faults & FAULT_MQ2) && !(faultsNotified & FAULT_MQ2)) {
    Blynk.logEvent("sensor_fault", String("MQ-2 reading stuck at ") + gasRaw + " - check wiring");
    faultsNotified |= FAULT_MQ2;
  }
}

/* =========================================================================
   ENHANCEMENT 02 - CONNECTION MONITORING
   ========================================================================= */
enum LinkChange { LINK_SAME, LINK_LOST, LINK_FIRST_UP, LINK_RESTORED };

LinkChange monitorLink(LinkMonitor &l, bool nowUp) {
  if (nowUp == l.up) return LINK_SAME;
  l.up = nowUp;
  if (!nowUp) {
    l.downSince = millis(); l.attempts = 0;
    Serial.printf("[NET] %s connection lost\n", l.name);
    return LINK_LOST;
  }
  if (!l.everUp) {
    l.everUp = true;
    Serial.printf("[NET] %s connected\n", l.name);
    return LINK_FIRST_UP;
  }
  l.outages++;
  l.lastOutageMs = millis() - l.downSince;
  Serial.printf("[NET] %s restored after %lu s (%lu attempts)\n",
                l.name, l.lastOutageMs / 1000, l.attempts);
  return LINK_RESTORED;
}

// Blynk.connect() blocks, so a fresh alarm takes priority for up to 60 s.
bool alarmHasPriority() {
  return stage == STAGE_CRITICAL && criticalSince &&
         millis() - criticalSince < CRITICAL_LOCKOUT;
}

void tryBlynkConnect() {
  blynkLink.attempts++;
  Serial.printf("[NET] Blynk reconnect attempt %lu\n", blynkLink.attempts);
  Blynk.connect(1500);
}

void pushStatus(bool force) {   // V5 status text, sent only when it changes
  if (!Blynk.connected()) return;
  String s;
  if (faults & FAULT_DHT) s = "DHT11 FAULT";
  if (faults & FAULT_MQ2) s += s.length() ? " | MQ-2 FAULT" : "MQ-2 FAULT";
  if (!s.length()) s = "Sensors OK";
  if (simActive) s = String("TEST MODE | ") + s;
  if (blynkLink.outages) { s += " | last outage "; s += blynkLink.lastOutageMs / 1000; s += " s"; }

  static String last;
  if (force || s != last) { Blynk.virtualWrite(VP_STATUS, s); last = s; }
}

/* =========================================================================
   TEST MODE - simulated gas for a safe demo (Blynk slider or Serial)
   Serial: "sim 950", "sim off", "spike 950" (spike = one sample only)
   ========================================================================= */
void startSimulation(int v) {
  if (v <= 0) {
    if (simActive) Serial.println("[TEST] Simulation off - using real MQ-2");
    simActive = false;
    return;
  }
  simGas = constrain(v, 1, 4095);
  simActive = true;
  simStarted = millis();
  Serial.printf("[TEST] Simulated gas = %d (auto-off in 5 min)\n", simGas);
}

void serviceSerial() {
  static String buf;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c != '\n' && c != '\r') { if (buf.length() < 30) buf += c; continue; }
    buf.trim(); buf.toLowerCase();
    if (buf.startsWith("spike")) {
      spikeValue = constrain((int)buf.substring(5).toInt(), 0, 4095);
      Serial.printf("[TEST] Next sample will read %d (one sample only)\n", spikeValue);
    } else if (buf.startsWith("sim")) {
      startSimulation(buf.substring(3).toInt());   // "sim off" -> 0 -> off
      if (!simActive && Blynk.connected()) Blynk.virtualWrite(VP_SIMGAS, 0);
    }
    buf = "";
  }
}

/* =========================================================================
   PERIODIC TASKS
   ========================================================================= */
void taskSense() {
  gasRaw   = readGas();
  soundVal = analogRead(SOUND_PIN);
  checkGasSensor(gasRaw);

  if (spikeValue >= 0)  { gasVal = spikeValue; spikeValue = -1; }
  else if (simActive)   gasVal = simGas;
  else                  gasVal = gasRaw;

  updateStage();
  applyStageOutputs();

  if (stage == STAGE_CRITICAL) { if (!criticalSince) criticalSince = millis(); }
  else criticalSince = 0;

  // Show progress toward a stage change, e.g. "(->ALERT 2/3)"
  char pending[24] = "";
  if (upCount)        snprintf(pending, sizeof pending, " (->%s %d/%d)", STAGE_NAME[upTarget], upCount, CONFIRM_SAMPLES);
  else if (downCount) snprintf(pending, sizeof pending, " (clear %d/%d)", downCount, CLEAR_SAMPLES);

  Serial.printf("Gas:%4d%s | Sound:%4d | Temp:%5.1fC | Hum:%5.1f%% | Stage:%s%s | Faults:%s | WiFi:%s Blynk:%s\n",
                gasVal, simActive ? "(sim)" : "", soundVal, tempC, humidity,
                STAGE_NAME[stage], pending,
                faults == 0 ? "none" : faults ==
