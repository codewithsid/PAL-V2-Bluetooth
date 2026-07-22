# PAL V2 Bluetooth Telemetry

This project turns an Adafruit ESP32 Feather V2 into a timestamped sensor telemetry node. It publishes JSON Lines simultaneously through the USB-C serial converter and a BLE GATT service. The included browser dashboard displays live values, downloads captured data as CSV, and sets the DS3231 to the computer's current UTC time.

## Wiring

| Device | Feather V2 connection |
| --- | --- |
| ICS43434 BCLK | A0 / GPIO26 |
| ICS43434 DOUT | A1 / GPIO25 |
| ICS43434 LRCL | **A5 / GPIO4** |
| ICS43434 SEL | GND or 3.3 V (firmware detects the active slot) |
| BME688, LSM6DSOX, DS3231 | STEMMA QT I2C chain (SDA GPIO22, SCL GPIO20) |

> **Required wiring correction:** A3 is GPIO39, which is input-only on the ESP32. The ICS43434 is an I2S slave, so the ESP32 must output LRCL. LRCL therefore cannot work on A3 and must be moved to an output-capable pin; this firmware uses A5/GPIO4.

Power the microphone at 3.3 V and connect all grounds. Tie its SEL/LR pin firmly to either GND or 3.3 V; do not leave it floating. Firmware captures both I2S slots and selects the active one. The BME688 defaults to I2C address `0x77` (the firmware also accepts `0x76`); the DS3231 address is `0x68`; the LSM6DSOX default address is `0x6A`.

## Sampling and transport

* The LSM6DSOX is configured for 104 Hz and acquired at 100 Hz. Ten IMU readings are packed in each 100 ms telemetry JSON record. The packet's `time_ms` is the beginning of that interval, and each IMU reading preserves its individual millisecond offset. `audio_time_ms` marks the end of the audio aggregation interval.
* The BME688 starts a non-blocking forced measurement once per second, so its gas-heater wait does not interrupt IMU or microphone acquisition. The microphone is sampled at 16 kHz. Every 100 ms, firmware calculates RMS energy and reports it as dBFS for a live 10 Hz sound-level plot. It also transports an 8 kHz signed 8-bit PCM preview for recording and playback. dBFS is relative to digital full scale; absolute dB SPL requires acoustic calibration against a known reference source.
* Every record has a Unix UTC timestamp anchored to the battery-backed DS3231. When the RTC battery has been lost, firmware starts it at the firmware build time and emits `clock_warning` until the dashboard sets it. A clock update is rejected if the DS3231 is unavailable.
* BLE uses a custom service `7f510001-5b8d-4a84-9c7c-a07142ab6001`. Data notifications are fragmented according to the client's negotiated ATT MTU (up to 180 payload bytes) and reassembled as newline-delimited JSON by the dashboard. Commands use the writable characteristic `7f510003-5b8d-4a84-9c7c-a07142ab6001`.

## Build, upload, and dashboard

Build and upload the `adafruit_feather_esp32_v2` environment with PlatformIO. Then open [dashboard/index.html](dashboard/index.html) in a current Chromium browser. Web Serial and Web Bluetooth require a secure context, so serve the project directory through `localhost` (for example, VS Code Live Server) rather than opening the file directly.

Choose **Connect USB** for the direct USB-C serial link or **Connect Bluetooth LE** and select `PAL-V2-Telemetry`. The most recently connected transport becomes active, preventing duplicate data if both links are present. BLE reconnects automatically after brief interruptions. Click **Set RTC to current UTC time** after connecting. **Download CSV** exports all readings collected during the dashboard session.

Click **Start recording** to begin retaining sensor rows and microphone PCM in browser memory. **Stop recording** ends the session, and **Play audio** plays the captured microphone preview. Live plots continue even when recording is stopped. Starting a new recording replaces the previous in-memory recording. CSV exports use UTC ISO 8601 date/time values such as `2026-07-17T17:39:18.123Z` instead of raw Unix milliseconds.

## References

* [Adafruit ESP32 Feather V2 pinouts](https://learn.adafruit.com/adafruit-esp32-feather-v2/pinouts)
* [Espressif I2S API](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/i2s.html)
* [Adafruit DS3231 Arduino usage](https://learn.adafruit.com/adafruit-ds3231-precision-rtc-breakout/arduino-usage)
* [Espressif Bluetooth API and NimBLE guidance](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/bluetooth/index.html)