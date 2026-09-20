# ESP32 Fire Detection System

An ESP32-based fire and safety monitoring system that uses multiple sensors to detect elevated gas levels and temperature conditions. The system provides local visual and audible alerts while also sending sensor data to a Blynk dashboard for remote monitoring and control.

## Features

* Gas detection using an MQ-2 sensor
* Temperature and humidity monitoring using a DHT11
* Sound sensor monitoring
* Five-stage LED status indicator
* Audible buzzer alarm for critical conditions
* Blynk dashboard integration
* Remote buzzer mute control
* Remote LED/lamp testing
* Automatic Wi-Fi and Blynk reconnection
* Real-time sensor telemetry
* Non-blocking buzzer operation
* Startup LED and buzzer tests

## Hardware

* ESP32
* MQ-2 Gas Sensor
* DHT11 Temperature & Humidity Sensor
* LM393 Sound Sensor
* 5 LEDs

  * 2 Green
  * 2 Yellow
  * 1 Red
* Active Piezo Buzzer
* Jumper wires
* Breadboard
* USB cable / ESP32 power source

## Pin Configuration

| Component                  | ESP32 Pin |
| -------------------------- | --------: |
| MQ-2 Analog Output         |   GPIO 34 |
| Sound Sensor Analog Output |   GPIO 35 |
| DHT11 Data                 |   GPIO 27 |
| Green LED 1                |    GPIO 4 |
| Green LED 2                |   GPIO 16 |
| Yellow LED 1               |   GPIO 17 |
| Yellow LED 2               |   GPIO 18 |
| Red LED                    |   GPIO 19 |
| Buzzer                     |   GPIO 25 |

## Detection Stages

The system evaluates the current sensor readings and assigns one of four states:

| Stage        | Condition                        | LED Status              |
| ------------ | -------------------------------- | ----------------------- |
| **Normal**   | Gas below warning threshold      | 2 green LEDs            |
| **Warning**  | Gas ≥ 700                        | 2 green + 2 yellow LEDs |
| **Alert**    | Gas ≥ 900                        | All LEDs                |
| **Critical** | Gas ≥ 1000 or temperature ≥ 34°C | All LEDs + buzzer       |

The critical condition uses an **OR** rule, meaning either a high gas reading or high temperature can trigger the critical state.

## Blynk Integration

The system connects to Blynk to provide remote monitoring and control.

### Device → Blynk

| Virtual Pin | Data                    |
| ----------- | ----------------------- |
| V0          | Gas reading             |
| V1          | Sound reading           |
| V2          | Temperature             |
| V3          | Humidity                |
| V4          | Current detection stage |

### Blynk → Device

| Virtual Pin | Function           |
| ----------- | ------------------ |
| V6          | Buzzer mute switch |
| V7          | Lamp test button   |

The system also sends Blynk events when the detection state increases, including critical fire alarms and elevated gas-level notifications.

## How It Works

1. The ESP32 initializes the connected sensors and output components.
2. A startup test activates each LED and briefly activates the buzzer.
3. The ESP32 connects to the configured Wi-Fi network.
4. Blynk is initialized for remote monitoring.
5. The MQ-2 and sound sensors are sampled periodically.
6. The DHT11 provides temperature and humidity readings.
7. Sensor readings are evaluated against the configured thresholds.
8. The appropriate LED stage is displayed.
9. If a critical condition is detected, the buzzer begins pulsing.
10. Sensor data is sent to the Blynk dashboard.
11. The system continues attempting to reconnect if Wi-Fi or Blynk becomes disconnected.

## Blynk Setup

Before uploading the program, replace the placeholder credentials in `fire-detection-system.ino`:

```cpp
#define BLYNK_TEMPLATE_ID   "YOUR_BLYNK_TEMPLATE_ID"
#define BLYNK_TEMPLATE_NAME "Fire Detection System"
#define BLYNK_AUTH_TOKEN    "YOUR_BLYNK_AUTH_TOKEN"

char ssid[] = "YOUR_WIFI_SSID";
char pass[] = "YOUR_WIFI_PASSWORD";
```

The ESP32 uses a **2.4 GHz Wi-Fi network**.

Create the required Blynk datastreams using the virtual pins listed above.

## Libraries

The project uses the following Arduino libraries:

* `WiFi.h`
* `WiFiClient.h`
* `BlynkSimpleEsp32.h`
* `DHT.h`

## Project Structure

```text
esp32-fire-detection-system/
│
├── fire-detection-system.ino
└── README.md
```

## Notes

The gas and temperature thresholds in the program are configured for this project and may need to be adjusted depending on the sensors, environment, and intended use.

This project is intended as an educational/prototype fire and safety monitoring system and should not be treated as a certified fire detection or emergency response system.

## Author

**Shan Christian M. Platon**

Computer Science Student
De La Salle Lipa

---

*Built with ESP32, Arduino, sensors, and Blynk.*
