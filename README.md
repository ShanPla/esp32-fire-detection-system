# Enhanced ESP32-Based Fire Detection System

**with Improved Hazard Detection and System Health Monitoring**

An ESP32 fire and safety monitor that reads gas, temperature, humidity and sound, shows a four-stage hazard level on LEDs and a buzzer, and reports to a Blynk dashboard.

This is the final-project continuation of our midterm prototype. It runs on the same hardware with the same wiring, and the improvements are all in software. The final version does two new things:

1. **Improved hazard detection.** Brief sensor fluctuations no longer trigger alerts, while a critical reading still sounds the alarm immediately.
2. **Sensor fault and connection monitoring.** The system notices when a sensor stops working or the network drops, and reports it on the Serial Monitor, on the Blynk dashboard and on the device itself.

---

## What changed from the midterm

| Situation | Midterm prototype | Final system |
|---|---|---|
| One-sample gas spike to 950 | Stage jumps to Alert immediately | Ignored and logged as a filtered transient |
| Gas holds at 950 | Alert | Alert after 3 samples (~1.2 s) |
| Gas or temperature reaches the Critical level | Critical + buzzer | Critical + buzzer, still **immediate** |
| Gas hovering right at a threshold | LEDs can flicker between stages | Hysteresis band and a 2 s clear delay keep the stage stable |
| DHT11 data wire unplugged | Old temperature kept forever, no warning | Fault after ~12.5 s, stale value dropped, gas-only mode, Blynk notification, LED1 blinks |
| MQ-2 signal stuck at 0 or 4095 | Not detected | Fault after 5 s, Blynk notification |
| Wi-Fi / Blynk drops | Retry messages on Serial only | Outage timed and counted, "back online" event, outage shown on dashboard |
| Fault happens while offline | — | Reported as soon as the connection returns |
| Lamp test | Used `delay()`, froze sensing for ~1.2 s | Non-blocking, sensing continues |
| Demonstration | Needed real gas or heat | Test mode injects simulated gas readings safely |

---

## Hardware

Unchanged from the midterm.

- ESP32 development board
- MQ-2 gas sensor
- DHT11 temperature and humidity sensor
- LM393 sound sensor
- 5 LEDs (2 green, 2 yellow, 1 red) with resistors
- Active piezo buzzer
- Breadboard, jumper wires, USB cable

## Pin configuration

| Component | ESP32 pin |
|---|---:|
| MQ-2 analog output | GPIO 34 |
| Sound sensor analog output | GPIO 35 |
| DHT11 data | GPIO 27 |
| Green LED 1 (also the fault indicator) | GPIO 4 |
| Green LED 2 | GPIO 16 |
| Yellow LED 1 | GPIO 17 |
| Yellow LED 2 | GPIO 18 |
| Red LED | GPIO 19 |
| Buzzer | GPIO 25 |

> **Optional:** a 100 kΩ resistor from GPIO 34 to GND makes a disconnected MQ-2 read near 0 reliably, so the MQ-2 fault check works consistently. Without it, a loose pin floats and may read random values.

---

## Enhancement 01: Improved hazard detection

### Stages

The thresholds are the same as the midterm.

| Stage | Condition | LEDs | Buzzer |
|---|---|---|---|
| 0 Normal | Gas below 700 | 2 green | Off |
| 1 Warning | Gas ≥ 700 | 2 green + 2 yellow | Off |
| 2 Alert | Gas ≥ 900 | All 5 | Off |
| 3 Critical | Gas ≥ 1000 **or** temperature ≥ 34 °C | All 5 | Pulsing |

### How the filter works

Gas is sampled every 400 ms. Each sample averages 8 ADC reads to reduce noise.

- **Escalating to Warning or Alert** requires 3 consecutive samples (about 1.2 s) above the current stage. If the readings in that run disagree (for example 750, 950, 960), the system moves to the lowest level all of them reached, which here is Warning.
- **Critical has no delay.** One sample at or above the Critical threshold sounds the alarm immediately. The filter never delays the alarm.
- **Stepping down** requires 5 consecutive samples (about 2 s) below a hysteresis band. To leave a stage, gas must fall 50 below its threshold, and temperature must fall 1 °C below 34 °C. This stops the LEDs and buzzer from flickering when a reading sits right at a threshold.
- Every ignored transient is printed to the Serial Monitor as a `[FILTER]` message and counted.

The Serial output shows each step as it happens:

```
Gas: 950 | ... | Stage:NORMAL (->ALERT 1/3) | Faults:none | WiFi:up Blynk:up
[FILTER] Ignored brief gas reading: peak 950 for 1 sample (3 needed) - stage stays NORMAL
...
>>> NORMAL -> ALERT (confirmed over 3 samples, gas=950)
>>> ALERT -> CRITICAL (immediate: gas=1100, temp=29.0C)
<<< CRITICAL -> NORMAL (cleared after 5 samples, gas=400)
```

## Enhancement 02: Sensor fault and connection monitoring

### Sensor faults

| Sensor | Fault condition | What the system does |
|---|---|---|
| DHT11 | 5 failed reads in a row (~12.5 s), or impossible values | Discards the stale reading, disables the temperature trip and keeps detecting gas. Recovers automatically on the next good read. |
| MQ-2 | Reading stuck at ≤ 20 or ≥ 4090 for 5 s | Raises a fault. Gas is still evaluated as read, so a saturated sensor keeps the system in Critical. |

Up to 4 failed DHT11 reads in a row are tolerated, and the last good value is kept during that time. DHT11s occasionally miss a read, and a single miss should not raise a fault.

While any fault is active:
- **Green LED 1 blinks.** This works even with no network.
- The Serial Monitor shows `[DIAG]`, `[FAULT]` and `[RECOVERED]` messages.
- The Blynk status text (V5) shows the fault, for example `DHT11 FAULT`.
- A `sensor_fault` event is sent once per fault. A fault that happens while offline is sent as soon as the connection returns, with how long ago it was detected.

### Connection monitoring

- Wi-Fi and Blynk are checked every second. Losses and recoveries are printed with timestamps.
- Reconnection is attempted every 15 s. When Wi-Fi returns, Blynk reconnects immediately instead of waiting for the next attempt.
- When the dashboard connection returns, a `connection_restored` event reports how long the device was offline, how many reconnect attempts it took, and whether any sensor is faulted. For example: *"Back online after 49 s offline (5 reconnect attempts). Sensors: DHT11 FAULT"*.
- The status text includes the duration of the last outage.
- During a new Critical alarm, reconnects pause for up to 60 s. `Blynk.connect()` blocks, and the alarm takes priority over the network.

---

## Blynk setup

### 1. Credentials

Replace the placeholders at the top of `fire-detection-system.ino`:

```cpp
#define BLYNK_TEMPLATE_ID   "YOUR_BLYNK_TEMPLATE_ID"
#define BLYNK_TEMPLATE_NAME "Fire Detection System"
#define BLYNK_AUTH_TOKEN    "YOUR_BLYNK_AUTH_TOKEN"

char ssid[] = "YOUR_WIFI_SSID";
char pass[] = "YOUR_WIFI_PASSWORD";
```

The ESP32 only connects to **2.4 GHz** Wi-Fi networks.

### 2. Datastreams

V0–V4, V6 and V7 are unchanged from the midterm. **V5 and V8 are new.**

| Pin | Name | Type | Range / units | Direction | Suggested widget |
|---|---|---|---|---|---|
| V0 | Gas | Integer | 0–4095 | Device → App | Gauge / Chart |
| V1 | Sound | Integer | 0–4095 | Device → App | Gauge / Chart |
| V2 | Temperature | Double | 0–50 °C | Device → App | Gauge |
| V3 | Humidity | Double | 0–100 % | Device → App | Gauge |
| V4 | Stage | Integer | 0–3 | Device → App | Label / LED |
| **V5** | **System Status** | **String** | — | Device → App | **Label** |
| V6 | Buzzer Mute | Integer | 0–1 | App → Device | Switch |
| V7 | Lamp Test | Integer | 0–1 | App → Device | Button (Push mode) |
| **V8** | **Test Gas** | **Integer** | **0–4095** | App → Device | **Slider** (0 = test mode off) |

### 3. Events

Create these in the template's events settings and enable notifications for the ones you want on your phone. The event codes must match exactly.

| Event code | Type | Sent when |
|---|---|---|
| `critical_fire_alarm` | Critical | Stage escalates to Critical |
| `elevated_gas_level` | Warning | Stage escalates to Warning or Alert |
| `sensor_fault` | Warning | **New:** DHT11 or MQ-2 fault detected |
| `connection_restored` | Info | **New:** device back online after an outage |

Events triggered in test mode start with `[TEST]`, so they can't be mistaken for real alarms.

> **Tip:** Blynk can also notify you when the device itself goes offline, which the device can't report while it's disconnected. Look for the built-in offline notification in the template's event settings.

---

## Test mode and Serial commands

Test mode replaces the MQ-2 reading with a simulated value, so the detection logic can be demonstrated without a flame or real gas. It switches itself off after 5 minutes, so the device can't be left in simulation by accident. While active, the status text reads `TEST MODE`.

Open the Serial Monitor at **115200 baud** with line ending set to **Newline**:

| Command | Effect |
|---|---|
| `help` | List commands |
| `status` | Full system health report (sensors, faults, Wi-Fi/RSSI, outages, test mode) |
| `sim 950` | Simulate a steady gas reading of 950 |
| `sim off` | Return to the real MQ-2 |
| `spike 950` | Inject a single sample of 950 to show the transient filter |
| `lamptest` | Run the LED/buzzer lamp test |

The **Test Gas slider (V8)** on the dashboard does the same as `sim`. Setting it to 0 turns test mode off.

---

## Demonstration guide

All tests are safe. **Do not use an open flame, lighter or real gas.**

### A. Hazard detection filter

1. `spike 950`: one sample of 950 is ignored and a `[FILTER]` message appears. The LEDs stay green.
2. `sim 950`: after about 1.2 s the stage becomes **Alert** and all LEDs turn on.
3. `sim 1100`: the stage becomes **Critical** immediately and the buzzer pulses.
4. `sim off`: after about 2 s the system returns to **Normal**.

On the midterm firmware, step 1 would have raised Alert immediately.

### B. DHT11 sensor fault

1. Briefly disconnect the DHT11 data wire (GPIO 27).
2. Watch `[DIAG] DHT11 read failed (1/5)`, then `(2/5)` and so on, up to `[FAULT] DHT11 not responding` after about 12 s.
3. Green LED 1 blinks, the dashboard status shows `DHT11 FAULT`, and a `sensor_fault` notification arrives.
4. Run `sim 1100` to show that gas detection still works during the fault.
5. Reconnect the wire. A `[RECOVERED]` message appears and the status returns to `Sensors OK`.

### C. Wi-Fi disconnection

1. Turn off the Wi-Fi hotspot or router.
2. Watch `[NET] Wi-Fi connection lost` and a reconnect attempt every 15 s. The LEDs and buzzer keep working offline.
3. Optional: unplug the DHT11 while offline, so the fault is queued.
4. Turn the hotspot back on. `[NET] Wi-Fi restored after N s` appears, Blynk reconnects, and a `connection_restored` event arrives, followed by any queued fault.

### D. Temperature trip (optional)

Warm the DHT11 gently with your fingers or warm breath until it reads ≥ 34 °C. The stage goes to **Critical**, then clears once the reading falls below 33 °C.

---

## Configuration

All tunable values are constants near the top of the sketch.

| Constant | Default | Meaning |
|---|---|---|
| `GAS_WARN` / `GAS_ALERT` / `GAS_ALARM` | 700 / 900 / 1000 | Stage thresholds (raw ADC) |
| `TEMP_ALARM` | 34.0 °C | Thermal trip |
| `CONFIRM_SAMPLES` | 3 | Samples needed to escalate to Warning/Alert |
| `CLEAR_SAMPLES` | 5 | Samples needed to step down |
| `GAS_HYST` / `TEMP_HYST` | 50 / 1.0 °C | Hysteresis band |
| `GAS_OVERSAMPLE` | 8 | ADC reads averaged per gas sample |
| `DHT_FAIL_LIMIT` | 5 | Failed DHT11 reads before a fault |
| `MQ2_RAIL_LOW` / `MQ2_RAIL_HIGH` | 20 / 4090 | MQ-2 stuck-signal limits |
| `MQ2_FAULT_MS` | 5000 ms | Time at a rail before an MQ-2 fault |
| `SIM_TIMEOUT_MS` | 300000 ms | Test mode auto-off |
| `T_SENSE` / `T_CLIMATE` / `T_RECONNECT` | 400 / 2500 / 15000 ms | Task intervals |

The thresholds were calibrated for our sensors and room. Re-check them if the environment changes. The MQ-2 also needs a few minutes of warm-up after power-on before its readings settle.

---

## Libraries and board setup

- **Board:** ESP32 Dev Module (esp32 by Espressif Systems, installed through the Arduino Boards Manager)
- **Libraries** (Arduino Library Manager):
  - `Blynk` by Volodymyr Shymanskyy
  - `DHT sensor library` by Adafruit
  - `Adafruit Unified Sensor` (required by the DHT library)
- `WiFi.h` and `WiFiClient.h` come with the ESP32 board package.

## Project structure

```
esp32-fire-detection-system/
├── fire-detection-system.ino
└── README.md
```

## Limitations

- This is an educational prototype. It is **not** a certified fire detection or emergency response system.
- While the DHT11 is faulted, the system detects gas only.
- The MQ-2 fault check catches a stuck signal. It cannot catch a sensor that is drifting but still producing readings.
- The device can't send anything while it's offline. Outages and faults are reported once it reconnects.

## Authors

**BSCS C4A, IoT Final Project**

- Shan Christian M. Platon
- Steephen L. Resurreccion
- Maria Fae Escobin

Computer Science, De La Salle Lipa

---

*Built with ESP32, Arduino, sensors and Blynk.*
