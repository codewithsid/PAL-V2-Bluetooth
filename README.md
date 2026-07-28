# PAL V2 Bluetooth Telemetry & Environmental Monitoring Node

This project transforms an **Adafruit ESP32 Feather V2** into an advanced, dual-core, multi-sensor telemetry node and web dashboard. It continuously samples motion (via hardware FIFO), environmental metrics (BME688 IAQ), digital I2S audio (with ADPCM compression), and battery health, streaming JSON telemetry over both **USB-C Serial** and **Bluetooth Low Energy (NimBLE)**.

Includes a state-of-the-art **Pro Obsidian UI Dashboard** featuring real-time signal plotting, live device local RTC clock, battery gauge, 16-bit PCM / 4-bit ADPCM audio decoding and playback, and CSV data export.

---

## Key Features & Architecture

* **FreeRTOS Dual-Core Architecture**:
  * **Core 0 (`telemetry_net_task`)**: Handles NimBLE stack events, Web Serial command processing, and I2S DMA microphone audio collection.
  * **Core 1 (`sensor_app_task`)**: Handles LSM6DSOX FIFO burst reads, BME688 forced measurements, DS3231 RTC drift sync, battery ADC monitoring, JSON telemetry serialization, and NeoPixel LED state machine.
* **Smart Audio Codec (Adaptive Resolution)**:
  * **USB Serial**: Streams uncompressed 16-bit 16 kHz signed PCM audio Base64 (`audio_pcm_s16_b64`) for maximum fidelity.
  * **BLE Wireless**: Compresses audio on-the-fly using a custom 4-bit IMA-ADPCM encoder (`audio_adpcm_b64`), achieving a 4:1 compression ratio to fit within BLE bandwidth limits.
* **Advanced Indoor Air Quality (G6EJD Model)**: Calculates a dynamic IAQ score (0-500) and eTVOC from the BME688 sensor using the robust G6EJD open-source model. It combines a 25% humidity contribution and a 75% gas resistance contribution (anchored at 100,000 Ω baseline resistance and 30% RH) for accurate indoor environmental sensing without closed-source blobs.
* **LSM6DSOX Hardware FIFO Burst**: Accelerometer and Gyroscope samples are accumulated in the sensor's hardware FIFO and burst-read over I2C to drastically reduce CPU bus overhead.
* **LiPo Battery Health**: Measures battery level and percentage via the internal voltage divider (`GPIO35`), streaming `"battery_v"` and `"battery_pct"`. 
* **Onboard RGB NeoPixel Status Priority (GPIO0)**:
  * 🔵 **Blue**: Active Bluetooth LE client connected.
  * 🔴 **Red**: Hardware sensor error or hazardous IAQ levels.
  * 🟡 **Amber**: DS3231 RTC clock sync required or Low Battery.
  * 🟢 **Green**: All systems operational.
* **Pro Web Dashboard**: Pure obsidian black theme (`#000000`), frosted glass cards (`backdrop-filter: blur(24px)`), dynamic IAQ status badges, 16-bit audio playback via a custom JavaScript ADPCM decoder, and CSV data export.

---

## Hardware Wiring & Pinout

| Device / Signal | Feather V2 Pin | Description / Protocol |
| :--- | :--- | :--- |
| **ICS43434 I2S BCLK** | **GPIO26** (A0) | I2S Bit Clock Output |
| **ICS43434 I2S DOUT** | **GPIO25** (A1) | I2S Data Input |
| **ICS43434 I2S LRCL** | **GPIO4** (A5) | I2S Left/Right Word Select Output |
| **ICS43434 SEL** | **GND or 3.3 V** | Channel Selection (Firmware auto-detects active slot) |
| **LiPo Battery Monitor** | **GPIO35** | Internal 100k:100k resistor divider to battery connector |
| **NeoPixel RGB Data** | **GPIO0** | Built-in WS2812B RGB LED Data Pin |
| **NeoPixel RGB Power** | **GPIO2** | NeoPixel & STEMMA QT Power Enable |
| **BME688 / LSM6DSOX / DS3231** | **STEMMA QT** | I2C Chain (SDA **GPIO22**, SCL **GPIO20**) |

---

## Telemetry JSON Protocol

### 1. Environment Packet (1 Hz)
```json
{
  "type": "environment",
  "time_ms": 1785185533307,
  "temperature_c": 26.96,
  "humidity_pct": 36.15,
  "pressure_hpa": 1009.13,
  "gas_ohms": 95167,
  "iaq": 45.2,
  "voc_ppm": 0.45,
  "eco2_ppm": 761.6,
  "battery_v": 4.12,
  "battery_pct": 91
}
```

### 2. Telemetry Packet (10 Hz)
Contains an array of unpacked IMU hardware FIFO samples `[offset_ms, ax, ay, az, gx, gy, gz]` alongside microphone sound level dBFS and Base64-encoded audio:
```json
{
  "type": "telemetry",
  "time_ms": 1785185533169,
  "imu": [
    [35, 3.5581, 5.3025, 6.7370, 0.1414, -0.0614, 0.0220],
    [44, 3.5163, 5.3216, 6.6987, 0.0666, -0.0512, 0.0190]
  ],
  "audio_time_ms": 1785185533269,
  "audio_samples": 1600,
  "audio_dbfs": -32.45,
  "audio_adpcm_b64": "..." 
}
```
*(Note: Uses `audio_pcm_s16_b64` over USB and `audio_adpcm_b64` over BLE).*

### 3. GATT Services & Characteristics
* **Service UUID**: `7f510001-5b8d-4a84-9c7c-a07142ab6001`
* **Data Characteristic (Notify/Read)**: `7f510002-5b8d-4a84-9c7c-a07142ab6001`
* **Command Characteristic (Write)**: `7f510003-5b8d-4a84-9c7c-a07142ab6001`

---

## Unit Testing & Verification

Run host unit tests using PlatformIO's native environment:

```bash
~/.platformio/penv/bin/pio test -e native
```

**Test Suite Cases (`test/test_main.cpp`)**:
* `test_base64_encoding`: Verifies Base64 string encoding.
* `test_pcm_quantization`: Tests PCM scaling.
* `test_dbfs_calculation`: Validates RMS dBFS energy math.
* `test_battery_percentage`: Verifies voltage-to-percentage conversion.
* `test_neopixel_status_priority`: Validates status color priority state machine.
* `test_battery_save_pulse_math`: Validates LED pulse math.
* `test_imu_fifo_unpacking`: Validates raw hardware FIFO unpacking.
* `test_adpcm_encoder_decoder_roundtrip`: Tests IMA-ADPCM compression fidelity.
* `test_iaq_calculation`: Validates the G6EJD IAQ model math logic.

---

## Firmware Build & Dashboard Usage

### 1. Build and Flash Firmware
Connect your ESP32 Feather V2 via USB-C and run:

```bash
~/.platformio/penv/bin/pio run -e adafruit_feather_esp32_v2 -t upload
```

### 2. Start Dashboard Web Server
Serve the `dashboard/` directory over HTTP (required for Web Serial & Web Bluetooth security rules):

```bash
python3 -m http.server 8000 --directory dashboard
```

Open **`http://localhost:8000`** in **Google Chrome** or **Microsoft Edge**.

### 3. Dashboard Controls
* **`🔌 Connect USB` / `📡 Connect BLE`**: Connects via Web Serial or Web Bluetooth.
* **`🛑 Disconnect`**: Performs safe disconnection of active transport.
* **`⚡ Sync Time`**: Sets DS3231 RTC clock to current computer UTC time.
* **`● Record` / `■ Stop`**: Captures sensor rows and PCM audio in browser memory.
* **`▶ Play Audio`**: Plays back recorded 16-bit PCM microphone audio (automatically decodes ADPCM if via BLE).
* **`💾 Export CSV`**: Exports captured session data as a formatted CSV file.