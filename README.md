# PAL V2 Bluetooth Telemetry & Environmental Monitoring Node

This project transforms an **Adafruit ESP32 Feather V2** into an advanced, dual-core, multi-sensor telemetry node and web dashboard. It continuously samples motion (via hardware FIFO), environmental metrics (BME688 IAQ), digital I2S audio (with ADPCM compression), and battery health, streaming JSON telemetry over both **USB-C Serial** and **Bluetooth Low Energy (NimBLE)**.

Includes a state-of-the-art **Pro Obsidian UI Dashboard** featuring real-time signal plotting, live device local RTC clock, battery gauge, 16-bit PCM / 4-bit ADPCM audio decoding and playback, and CSV data export.

---

## Key Features & Architecture

* **FreeRTOS Dual-Core Architecture**:
  * **Core 0 (`telemetry_net_task`)**: Handles NimBLE stack events, Web Serial command processing, and I2S DMA microphone audio collection.
  * **Core 1 (`sensor_app_task`)**: Handles LSM6DSOX FIFO burst reads, BME688 forced measurements, DS3231 RTC drift sync, battery ADC monitoring, JSON telemetry serialization, and NeoPixel LED state machine.
* **I2C Bus & IMU FIFO Optimizations**:
  * **400 kHz Fast Mode**: STEMMA QT bus operates in I2C Fast Mode (`Wire.setClock(400000)`), reducing bus transaction latency by 75%.
  * **Batched FIFO Burst Reads**: LSM6DSOX accelerometer & gyroscope hardware FIFO samples are burst-read in chunks of up to 18 words (126 bytes), cutting I2C Start/Stop transactions per cycle by over 90% and avoiding I2S audio buffer underruns on Core 0.
* **Hardware-Anchored RTC Time Synchronization**: Anchors absolute UTC time from the DS3231 RTC once at boot/sync (`clockBaseMs`), then interpolates sub-millisecond timestamps using the ESP32's hardware `millis()` counter. This provides 0 I2C bus overhead during 100 Hz data acquisition and handles 32-bit rollover safely.
* **Smart Audio Codec (Adaptive Resolution)**:
  * **USB Serial**: Streams uncompressed 16-bit 16 kHz signed PCM audio Base64 (`audio_pcm_s16_b64`) for maximum fidelity.
  * **BLE Wireless**: Compresses audio on-the-fly using a custom 4-bit IMA-ADPCM encoder (`audio_adpcm_b64`), achieving a 4:1 compression ratio to fit within BLE bandwidth limits.
* **Advanced Indoor Air Quality (G6EJD Model)**: Calculates a dynamic IAQ score (0-500) and eTVOC from the BME688 sensor using the robust G6EJD open-source model. It combines a 25% humidity contribution and a 75% gas resistance contribution (anchored at 100,000 Ω baseline resistance and 30% RH) for accurate indoor environmental sensing without closed-source blobs.
* **LiPo Battery Health**: Measures battery level and percentage via the internal voltage divider (`GPIO35`), streaming `"battery_v"` and `"battery_pct"`. 
* **Onboard RGB NeoPixel Status Priority (GPIO0)**:
  * 🔵 **Blue**: Bluetooth LE advertising or connection/subscription handshake in progress.
  * 🟢 **Green**: Bluetooth notifications are subscribed and telemetry is streaming.
  * 🔴 **Red**: Hardware sensor error or hazardous IAQ levels.
  * 🟡 **Amber**: DS3231 RTC clock sync required or Low Battery.
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

### 3. Audio Spectrum Packet (1 Hz initial rate, 5 Hz maximum)

The optional spectrum stream is a separate packet, so existing telemetry and environment consumers do not need to change. It uses the real 16 kHz microphone sample rate and contains exactly 32 finite dBFS values:

```json
{
  "type": "audio_spectrum",
  "time_ms": 1785185533369,
  "sample_rate_hz": 16000,
  "fft_size": 256,
  "min_hz": 0,
  "max_hz": 8000,
  "scale": "linear",
  "audio_fft": [-100.0, -100.0, -99.4, -96.1, -71.8, -54.2, -31.6, -47.5, -79.2, -91.0, -96.3, -98.4, -99.1, -99.5, -99.8, -100.0, -100.0, -100.0, -100.0, -100.0, -100.0, -100.0, -100.0, -100.0, -100.0, -100.0, -100.0, -100.0, -100.0, -100.0, -100.0, -100.0]
}
```

`audio_fft` uses a 256-sample Hann window and a single-sided radix-2 FFT. The 129 bins from DC through Nyquist are assigned to 32 equal-width linear bands (250 Hz per band at 16 kHz). Each band is the RMS amplitude derived from the **mean power** of its bins, expressed in dBFS. Values are clamped to `-100.0` through `0.0` dBFS; zero or non-finite results become `-100.0`. A packet is skipped until a complete 256-sample window is available.

`AUDIO_SPECTRUM_ENABLED` in `src/main.cpp` is the compile-time feature flag and defaults to `true`. `AUDIO_SPECTRUM_INTERVAL_MS` initially defaults to 1000 ms (1 packet/s) for hardware validation; a compile-time assertion prevents settings below 200 ms (more than 5 packets/s). Setting the enable flag to `false` removes the capture, FFT, JSON, and transmission work while leaving all prior packet behavior unchanged.

A typical compact packet is about 330-370 bytes, depending on the timestamp and values. At the initial 1 Hz rate this is approximately 0.33-0.37 kB/s of JSON payload before BLE link-layer overhead (1.7-1.9 kB/s at the enforced 5 Hz maximum). With the requested MTU 247 and the firmware's 180-byte chunk cap, most packets require two or three notifications. Persistent RAM cost is 512 bytes for the sample window plus a few counters; peak task-stack working storage is approximately 3 KiB for the FFT arrays, snapshot, band values, and JSON buffer. Processing is one 256-point FFT per configured interval and does not alter I2S DMA timing.

### 4. GATT Services & Characteristics
* **Service UUID**: `7f510001-5b8d-4a84-9c7c-a07142ab6001`
* **Data Characteristic (Notify/Read)**: `7f510002-5b8d-4a84-9c7c-a07142ab6001`
* **Command Characteristic (Write)**: `7f510003-5b8d-4a84-9c7c-a07142ab6001`

BLE messages use UTF-8 JSON Lines framing. Every producer submits a complete JSON object to the owned-message queue, which appends exactly one `\n`. The single network task dequeues one message and sends all of its MTU-sized fragments—including the trailing newline—before it starts another message. App Inventor must concatenate `StringsReceived` fragments until the newline arrives, then decode the complete JSON object. After notification registration succeeds, PAL emits a `status` packet with `state` set to `ble_ready`; sending `{"cmd":"ping"}` returns a `status` packet with `state` set to `pong`.

Status/command responses have highest queue priority, telemetry and environment packets are next, and spectrum packets are last. The queue holds 16 owned messages. When full, any pending spectrum packet is evicted first; a new spectrum packet can never displace telemetry, environment, or responses. If no spectrum is pending, a response may displace the oldest normal packet; otherwise a full queue rejects the incoming lower-priority packet. Each enqueued buffer remains owned and immutable until the network task finishes every fragment and releases it. A persistent notification failure disconnects BLE rather than allowing later packets to follow a partial JSON line.

For MIT App Inventor, request MTU 247 before calling `RegisterForStrings` when possible. The default 23-byte MTU is supported, but requires many more notification fragments.

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

