/* ---- Blynk template credentials ----
   These three lines MUST come before any #include, or the library will not
   compile.                                                                */
#define BLYNK_TEMPLATE_ID   "YOUR_BLYNK_TEMPLATE_ID"
#define BLYNK_TEMPLATE_NAME "Fire Detection System"
#define BLYNK_AUTH_TOKEN    "YOUR_BLYNK_AUTH_TOKEN"

#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <DHT.h>

/* ---------------- WI-FI CREDENTIALS */
char ssid[] = "YOUR_WIFI_SSID";
char pass[] = "YOUR_WIFI_PASSWORD";
// Note: the ESP32 radio is 2.4 GHz only. A 5 GHz-only SSID will never connect.

/* ---------------- PIN ASSIGNMENTS ---------------- */
const int MQ2_PIN   = 34;   // MQ-2 analog out   (ADC1 - safe with Wi-Fi on)
const int SOUND_PIN = 35;   // LM393 analog out  (ADC1 - safe with Wi-Fi on)
const int DHT_PIN   = 27;   // DHT11 single-wire data

const int LED1 = 4;         // GREEN
const int LED2 = 16;        // GREEN
const int LED3 = 17;        // YELLOW
const int LED4 = 18;        // YELLOW
const int LED5 = 19;        // RED

const int BUZZER = 25;      // active piezo buzzer - the project actuator

const int leds[5] = {LED1, LED2, LED3, LED4, LED5};

/* ---------------- BLYNK VIRTUAL PINS ----------------
   Device -> App (telemetry)        App -> Device (control)
   V0 gas raw       V3 humidity     V6 buzzer mute switch
   V1 sound raw     V4 stage index  V7 lamp test button
   V2 temperature                          */
#define VP_GAS       V0
#define VP_SOUND     V1
#define VP_TEMP      V2
#define VP_HUM       V3
#define VP_STAGE     V4
#define VP_MUTE      V6
#define VP_LAMPTEST  V7

/* ---------------- DHT CONFIGURATION ---------------- */
#define DHTTYPE DHT11
DHT dht(DHT_PIN, DHTTYPE);

/* ---------------- CALIBRATED THRESHOLDS ---------------- */
const int   GAS_WARN   = 700;    // Stage 2 - yellow caution tier engages
const int   GAS_ALERT  = 900;    // Stage 3 - red lamp engages
const int   GAS_ALARM  = 1000;   // Stage 4 - audible alarm
const float TEMP_ALARM = 34.0;   // Independent thermal trip, deg C

/* ---------------- TIMING ---------------- */
const unsigned long T_SENSE    = 400;    // gas + sound + arbitration
const unsigned long T_CLIMATE  = 2000;   // DHT11 tops out near 0.5 Hz
const unsigned long T_PUSH     = 2000;   // telemetry to Blynk
const unsigned long T_RECONNECT = 15000; // network retry
const unsigned long BUZZ_ON    = 150;    // matches the documented pulse
const unsigned long BUZZ_OFF   = 400;

/* ---------------- STATE ---------------- */
enum Stage { STAGE_NORMAL = 0, STAGE_WARNING = 1, STAGE_ALERT = 2, STAGE_CRITICAL = 3 };
const char* STAGE_NAME[4] = { "NORMAL", "WARNING", "ALERT", "CRITICAL" };

Stage stage     = STAGE_NORMAL;
Stage lastStage = STAGE_NORMAL;
unsigned long criticalSince = 0;   // millis() when Critical was entered, 0 if not

int   gasVal    = 0;
int   soundVal  = 0;
float tempC     = NAN;
float humidity  = NAN;

bool  buzzerMuted = false;   // set from the Blynk app (V6)
bool  buzzOn      = false;
unsigned long buzzMark = 0;

BlynkTimer timer;

/* =========================================================================
   OUTPUT HELPERS
   ========================================================================= */
void setLamps(bool a, bool b, bool c, bool d, bool e) {
  digitalWrite(LED1, a);   // green
  digitalWrite(LED2, b);   // green
  digitalWrite(LED3, c);   // yellow
  digitalWrite(LED4, d);   // yellow
  digitalWrite(LED5, e);   // red
}

void applyStageOutputs() {
  switch (stage) {
    case STAGE_CRITICAL:                       // all five lit, buzzer pulsing
    case STAGE_ALERT:    setLamps(1,1,1,1,1); break;
    case STAGE_WARNING:  setLamps(1,1,1,1,0); break;   // 2 green + 2 yellow
    default:             setLamps(1,1,0,0,0); break;   // green pair only
  }
}

/* Non-blocking buzzer. Called from loop() every pass, so the pulse cadence
   is independent of the sampling period and of Blynk traffic.            */
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

/* =========================================================================
   DECISION ALGORITHM
   Priority-ordered cascade, recomputed from the current sample set each
   cycle. Critical is entered on a logical OR, so heat without gas - or gas
   without heat - still reaches the audible stage.
   ========================================================================= */
Stage evaluateStage(int gas, float t) {
  if (gas >= GAS_ALARM || (!isnan(t) && t >= TEMP_ALARM)) return STAGE_CRITICAL;
  if (gas >= GAS_ALERT)                                   return STAGE_ALERT;
  if (gas >= GAS_WARN)                                    return STAGE_WARNING;
  return STAGE_NORMAL;
}

/* =========================================================================
   PERIODIC TASKS
   ========================================================================= */
void taskSense() {
  gasVal   = analogRead(MQ2_PIN);
  soundVal = analogRead(SOUND_PIN);

  stage = evaluateStage(gasVal, tempC);
  applyStageOutputs();

  // Push a Blynk event only on an upward transition, never every cycle.
  // Log a Blynk event on an upward transition only, never every cycle.
  if (stage > lastStage) {
    Serial.printf(">>> %s -> %s at gas=%d\n",
                  STAGE_NAME[lastStage], STAGE_NAME[stage], gasVal);
    if (Blynk.connected()) {
      if (stage == STAGE_CRITICAL) {
        Blynk.logEvent("critical_fire_alarm",
                       String("CRITICAL - gas ") + gasVal +
                       " / temp " + (isnan(tempC) ? String("n/a") : String(tempC, 1)) + "C");
      } else if (stage == STAGE_WARNING) {
        Blynk.logEvent("elevated_gas_level", String("Elevated gas level: ") + gasVal);
      }
    }
  }

  // Track how long we have been Critical (see taskReconnect).
  if (stage == STAGE_CRITICAL) {
    if (criticalSince == 0) criticalSince = millis();
  } else {
    criticalSince = 0;
  }

  lastStage = stage;

  Serial.printf("Gas:%4d | Sound:%4d | Temp:%5.1fC | Hum:%5.1f%% | Stage:%-8s | Blynk:%s\n",
                gasVal, soundVal, tempC, humidity, STAGE_NAME[stage],
                Blynk.connected() ? "up" : "down");
}

void taskClimate() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();

  // A failed read degrades the system to gas-only arbitration rather than
  // propagating an invalid comparison into the thermal trip.
  if (!isnan(t) && !isnan(h)) {
    tempC = t;
    humidity = h;
  } else {
    Serial.println("DHT11: read error, retaining previous sample");
  }
}

void taskPush() {
  if (!Blynk.connected()) return;

  /*  Send-on-change. The free tier allows 100k messages, and writing six
      datastreams every second burns that in about 4.5 hours. Temperature,
      humidity and stage rarely change, so only the gas and sound channels
      cost anything in practice. A full resend every 60 s keeps the dashboard
      correct if a write is ever dropped.                                  */
  static int   pGas = -1, pSound = -1, pStage = -1;
  static float pTemp = -999, pHum = -999;
  static unsigned long lastFull = 0;

  bool full = (millis() - lastFull > 60000);
  if (full) lastFull = millis();

  if (full || gasVal   != pGas)   { Blynk.virtualWrite(VP_GAS,   gasVal);   pGas   = gasVal; }
  if (full || soundVal != pSound) { Blynk.virtualWrite(VP_SOUND, soundVal); pSound = soundVal; }
  if (full || (int)stage != pStage) { Blynk.virtualWrite(VP_STAGE, (int)stage); pStage = (int)stage; }

  if (!isnan(tempC)    && (full || tempC    != pTemp)) { Blynk.virtualWrite(VP_TEMP, tempC); pTemp = tempC; }
  if (!isnan(humidity) && (full || humidity != pHum))  { Blynk.virtualWrite(VP_HUM,  humidity); pHum = humidity; }
}

const unsigned long CRITICAL_LOCKOUT = 60000;   // 60 s

void taskReconnect() {
  /*  Normally the alarm takes priority over the link, because Blynk.connect()
      blocks for its timeout. But TEMP_ALARM is close to ambient, so a warm
      room or a drifting DHT11 can hold the node in Critical indefinitely -
      and an unconditional early return would then mean the device NEVER
      reconnects and never appears in the app. So the lockout expires: after
      60 s of continuous Critical, the link wins.                          */
  if (stage == STAGE_CRITICAL && criticalSince != 0 &&
      (millis() - criticalSince) < CRITICAL_LOCKOUT) return;

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi down, retrying...");
    WiFi.begin(ssid, pass);
    return;
  }
  if (!Blynk.connected()) {
    Serial.println("Blynk down, reconnecting...");
    Blynk.connect(1500);
  }
}

/* =========================================================================
   BLYNK APP -> DEVICE HANDLERS
   ========================================================================= */
BLYNK_WRITE(VP_MUTE) {                 // switch widget
  buzzerMuted = (param.asInt() == 1);
  Serial.printf("Buzzer %s from app\n", buzzerMuted ? "MUTED" : "ARMED");
  if (buzzerMuted && buzzOn) { noTone(BUZZER); buzzOn = false; }
}

BLYNK_WRITE(VP_LAMPTEST) {             // momentary button widget (Push mode)
  if (param.asInt() != 1) return;
  Serial.println("Lamp test requested from app");
  for (int i = 0; i < 5; i++) { digitalWrite(leds[i], HIGH); delay(150); }
  delay(300);
  for (int i = 0; i < 5; i++) digitalWrite(leds[i], LOW);
  tone(BUZZER, 2000); delay(100); noTone(BUZZER);
  applyStageOutputs();                 // restore the real display
}

BLYNK_CONNECTED() {
  Serial.println("Blynk connected");
  Blynk.syncVirtual(VP_MUTE);          // restore the mute state after a reboot
  taskPush();                          // send a snapshot immediately
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

  Serial.println("\n--- System Initializing ---");

  // Sequential startup sweep - verifies all five output channels
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

  // First climate sample before the loop starts, so the thermal trip is
  // armed on the very first arbitration pass.
  taskClimate();

  timer.setInterval(T_SENSE,      taskSense);
  timer.setInterval(T_CLIMATE,    taskClimate);
  timer.setInterval(T_PUSH,       taskPush);
  timer.setInterval(T_RECONNECT,  taskReconnect);

  Serial.println("--- Fire & Safety Monitoring Active ---");
}

/* =========================================================================
   LOOP - no delay() anywhere below this line
   ========================================================================= */
void loop() {
  if (Blynk.connected()) Blynk.run();
  timer.run();
  serviceBuzzer();
}