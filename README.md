# PAL V2 Bluetooth Telemetry & Environmental Monitoring Node

This project transforms an **Adafruit ESP32 Feather V2** into an advanced, multi-sensor telemetry node and web dashboard. It continuously samples motion (accelerometer/gyroscope), environmental metrics (temperature, humidity, pressure, gas resistance), digital audio, and battery health, streaming JSON telemetry over both **USB-C Serial** and **Bluetooth Low Energy (NimBLE)**.

Includes a state-of-the-art **Apple Pro Obsidian UI Dashboard** featuring real-time signal plotting, live device local RTC clock, battery gauge, PCM audio playback, and CSV data export.

---

## Key Features & Hardware Capabilities

* **LiPo Battery Health & Power Monitoring**: Measures battery level and percentage via internal voltage divider pin (`GPIO35`). Maps $3.30\text{ V} \rightarrow 0\%$ up to $4.20\text{ V} \rightarrow 100\%$, streaming `"battery_v"` and `"battery_pct"` in telemetry.
* **Onboard RGB NeoPixel Status LED**: WS2812B RGB NeoPixel on `GPIO0` (powered via `GPIO2`):
  * **Boot Self-Test**: 1.2-second color sweep (Red ➔ Green ➔ Blue ➔ Yellow ➔ Cyan ➔ Magenta ➔ White ➔ Off) on startup.
  * **Real-time Status Priority State Machine**:
    * 🔵 **Blue (`#0000FF`)**: Active Bluetooth LE client connected.
    * 🔴 **Red (`#FF0000`)**: Hardware sensor error or BME688 Gas Hazard ($<20\text{ k}\Omega$ VOC drop).
    * 🟡 **Amber (`#FFA500`)**: DS3231 RTC clock sync required or Low Battery ($<20\%$ / $<3.48\text{ V}$).
    * 🟢 **Green (`#00FF00`)**: All systems operational, battery healthy, clock synchronized.
* **Battery Save Mode (5-Second LED Pulse)**: Activated via `toggle_battery_save` command or dashboard button. Preserves 100% full sensor data acquisition and USB/BLE transport while pulsating the status LED (600 ms sine envelope) once every 5 seconds (5000 ms), significantly reducing LED power draw.
* **Parallel BLE Safety Disconnection**: Asynchronous non-blocking watchdog (`performSafeBleDisconnect()`) handles unexpected link loss, out-of-range peer drops, or manual disconnects. Instantly stops active recording, clears GATT locks, and auto-restarts BLE advertising (`PAL-V2-Telemetry`).
* **Non-Blocking Serial Telemetry**: `sendLine()` prioritizes USB serial execution without stalling on BLE timeouts, guaranteeing zero data stream stuttering or packet loss.
* **Precision RTC Unix Timestamps**: Hardware DS3231 RTC anchors all telemetry packets to millisecond Unix UTC timestamps. Drift offset relative to computer clock is calculated in real-time.
* **Apple Pro Web Dashboard**: Pure obsidian black theme (`#000000`), frosted glass cards (`backdrop-filter: blur(24px)`), titanium pill controls, color-matched signal legends (**X**: Cyan `#38bdf8`, **Y**: Amber `#fbbf24`, **Z**: Rose `#f43f5e`), and 3.5s transient status notifications.
* **Native Unit Test Suite**: Unity test runner verifying Base64 encoding, 32-to-8 bit PCM audio quantization (`raw >> 24`), audio dBFS math, battery percentage mapping, NeoPixel status priority evaluation, and 5s pulse math.

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

> [!IMPORTANT]
> **Wiring Requirement:** Pin A3 is GPIO39, which is input-only on the ESP32. The ICS43434 requires the ESP32 to drive LRCL, so LRCL **must** be connected to an output-capable pin (**GPIO4 / A5**). Power the microphone at 3.3 V and tie its SEL pin firmly to GND or 3.3 V.

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
  "battery_v": 4.12,
  "battery_pct": 91
}
```

### 2. Telemetry Packet (10 Hz)
Contains 10 array samples of 100 Hz IMU readings `[offset_ms, ax, ay, az, gx, gy, gz]` alongside microphone sound level dBFS and Base64-encoded 8 kHz 8-bit PCM audio:
```json
{
  "type": "telemetry",
  "time_ms": 1785185533169,
  "imu": [
    [35, 3.5581, 5.3025, 6.7370, 0.1414, -0.0614, 0.0220],
    [44, 3.5163, 5.3216, 6.6987, 0.0666, -0.0512, 0.0190]
  ],
  "audio_time_ms": 1785185533269,
  "audio_samples": 800,
  "audio_dbfs": -32.45,
  "audio_pcm_s8_b64": "..."
}
```

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
* `test_pcm_quantization`: Tests 32-to-8 bit PCM sample shifting (`raw >> 24`) and clipping protection.
* `test_dbfs_calculation`: Validates RMS dBFS energy math and $-120\text{ dBFS}$ floor.
* `test_battery_percentage`: Verifies voltage-to-percentage conversion ($3.3\text{V} \rightarrow 0\%$, $4.2\text{V} \rightarrow 100\%$).
* `test_neopixel_status_priority`: Validates status color priority state machine (Blue, Red, Amber, Green).
* `test_battery_save_pulse_math`: Validates 5-second pulse period, 300 ms peak intensity, and off phase.

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
* **`🔌 Connect USB`**: Connects via Web Serial at 115200 baud.
* **`📡 Connect BLE`**: Connects via Web Bluetooth (automatically disconnects USB to prevent port conflict).
* **`🛑 Disconnect`**: Performs safe disconnection of active transport, stops streaming, and restarts BLE advertising.
* **`🔋 Battery Save`**: Toggles 5-second pulsating LED power save mode while retaining continuous data acquisition.
* **`⚡ Sync Time`**: Sets DS3231 RTC clock to current computer UTC time.
* **`● Record` / `■ Stop`**: Captures sensor rows and PCM audio in browser memory.
* **`▶ Play Audio`**: Plays back recorded 8 kHz PCM microphone audio.
* **`💾 Export CSV`**: Exports captured session data as a formatted CSV file with ISO 8601 UTC timestamps.

---

## References

* [Adafruit ESP32 Feather V2 Documentation & Pinouts](https://learn.adafruit.com/adafruit-esp32-feather-v2/pinouts)
* [Espressif ESP-IDF I2S API Guide](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/i2s.html)
* [Adafruit DS3231 Precision RTC Breakout](https://learn.adafruit.com/adafruit-ds3231-precision-rtc-breakout/arduino-usage)
* [NimBLE-Arduino BLE Stack Documentation](https://github.com/h2zero/NimBLE-Arduino)