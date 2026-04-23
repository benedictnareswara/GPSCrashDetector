# GPS Crash Detector (VestMicro)

Embedded project for crash-event detection on an Arduino Mega 2560 and UART forwarding to an ESP32 receiver.

## Overview

The Mega firmware reads MPU6050 accelerometer data and GPS NMEA input, evaluates crash-related thresholds, drives buzzer/button state logic, and emits event packets over UART.

The ESP32 firmware receives and parses the Mega packets for downstream handling/logging.

## Repository Layout

- `src/main.cpp` - Main firmware for Arduino Mega 2560
- `platformio.ini` - PlatformIO environment for Mega (`megaatmega2560`)
- `firmware/esp32/src/main.cpp` - ESP32 UART receiver firmware
- `firmware/esp32/platformio.ini` - PlatformIO environment for ESP32 (`esp32dev`)
- `PROTOCOL.md` - UART packet format and state model
- `examples/esp32_uart_receiver/` - Arduino IDE example receiver sketch

## Protocol

Current event packet format:

`EVT,<seq>,<event>,<valid>,<lat>,<lon>,<age_ms>,<tilt_deg>,<accel_g>`

Legacy packet still supported by parser:

`GPS,<seq>,<valid>,<lat>,<lon>,<age_ms>`

See `PROTOCOL.md` for full details.

## Trigger Logic (Mega)

- Crash trigger when `tilt >= 35.0` OR `accel >= 1.35g`
- Cancel window: 10 seconds after `CRASH_START`
- Cooldown after crash exit: 3 seconds
- Manual standby button flow:
  - Press once -> `MANUAL`, buzzer ON
  - Press again -> `CLEAR`, buzzer OFF

## Hardware Notes

- Mega <-> ESP32 link uses UART2 at `115200`
- ESP32 default UART2 pins in this project:
  - RX2: GPIO16
  - TX2: GPIO17
- Ensure voltage-level compatibility between Mega (5V) and ESP32 (3.3V), especially Mega TX -> ESP32 RX path.

## Build and Upload

Prerequisite: [PlatformIO Core](https://platformio.org/install/cli) installed.

### Mega Firmware

From repository root:

```bash
pio run
pio run -t upload
pio device monitor -b 115200
```

### ESP32 Firmware

From `firmware/esp32`:

```bash
pio run
pio run -t upload
pio device monitor -b 115200
```

## Typical Bring-Up Flow

1. Flash Mega firmware from repository root.
2. Flash ESP32 firmware from `firmware/esp32`.
3. Open serial monitors for both boards at `115200`.
4. Verify that ESP32 prints parsed `EVT`/`GPS` packets from Mega.

## GitHub

Repository target:

`https://github.com/benedictnareswara/GPSCrashDetector`

If this local folder has not been initialized yet:

```bash
git init
git add .
git commit -m "Initial commit"
git branch -M main
git remote add origin https://github.com/benedictnareswara/GPSCrashDetector.git
git push -u origin main
```

If `origin` already exists and points somewhere else:

```bash
git remote set-url origin https://github.com/benedictnareswara/GPSCrashDetector.git
git push -u origin main
```
