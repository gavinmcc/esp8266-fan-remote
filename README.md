# esp8266-fan-remote

WiFi-controlled ceiling fan remote using an ESP8266 and CC1101 433 MHz transceiver.
Replays captured OOK RF codes to control a RHINE UC7070T ceiling fan over HTTP.

## Hardware

| Component | Notes |
|-----------|-------|
| ESP8266 | Generic board (e.g. Wemos D1 Mini, NodeMCU) |
| CC1101 E07-M1010 | 433 MHz OOK transceiver module |

## Wiring

| CC1101 Pin | ESP8266 GPIO | NodeMCU Label |
|------------|-------------|---------------|
| VCC        | 3.3V        |               |
| GND        | GND         |               |
| MOSI       | GPIO13      | D7            |
| MISO       | GPIO12      | D6            |
| SCK        | GPIO14      | D5            |
| CSN        | GPIO15      | D8            |
| GDO0       | —           | not connected |

> **Note:** GDO0 is unused. TX is handled via CC1101 FIFO mode — data travels over SPI, not GDO0.

## Dependencies

Install via Arduino Library Manager:

- [SmartRC-CC1101-Driver-Lib](https://github.com/LSatan/SmartRC-CC1101-Driver-Lib) — search `ELECHOUSE CC1101`
- ESP8266 board support — search `esp8266` in Boards Manager (Boards Manager URL: `http://arduino.esp8266.com/stable/package_esp8266com_index.json`)

## Setup

1. Open `fan_remote/fan_remote.ino` in Arduino IDE
2. Set board to **Generic ESP8266 Module**
3. Edit the WiFi credentials near the top of the file:
   ```cpp
   const char* SSID     = "your-ssid";
   const char* PASSWORD = "your-password";
   ```
4. Flash to the ESP8266
5. Open Serial Monitor (115200 baud) — the assigned IP is printed on boot

## Usage

Navigate to `http://<ip>/` for the web UI, or call endpoints directly:

| Endpoint    | Action              |
|-------------|---------------------|
| `/`         | Web control page    |
| `/hi`       | Fan high speed      |
| `/med`      | Fan medium speed    |
| `/low`      | Fan low speed       |
| `/off`      | Fan off             |
| `/light`    | Toggle light        |
| `/carrier`  | 3s RF carrier test  |

The `/carrier` endpoint transmits a continuous 433.92 MHz carrier for 3 seconds.
Use it to verify RF emission: the physical remote should fail to operate the fan
during that window if the CC1101 is transmitting.

## Target Fan

RHINE UC7070T ceiling fan, DIP switch address `E7 80 29`.
See [SPEC.md](SPEC.md) for full protocol details and how to adapt for other addresses.

## Status

Working: ESP8266 WiFi, HTTP server, CC1101 SPI communication, FIFO streaming (no underflows).  
Unconfirmed: RF emission from the specific CC1101 module tested. Replacement module on order.
