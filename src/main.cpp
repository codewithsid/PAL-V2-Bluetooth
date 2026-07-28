#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>
#include <Adafruit_BME680.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_NeoPixel.h>
#include <NimBLEDevice.h>
#include <driver/i2s.h>
#include <freertos/queue.h>
#include <ctype.h>
#include <errno.h>
#include <math.h>

// Feather V2 pin labels: A0=GPIO26, A1=GPIO25, A5=GPIO4.
// A3/GPIO39 is input-only and cannot generate LRCL. Rewire LRCL to A5.
// Both I2S slots are captured, so the ICS43434 SEL pin may select either slot.
constexpr gpio_num_t MIC_BCLK = GPIO_NUM_26;
constexpr gpio_num_t MIC_DOUT = GPIO_NUM_25;
constexpr gpio_num_t MIC_LRCL = GPIO_NUM_4;
constexpr gpio_num_t PIN_BATTERY = GPIO_NUM_35;
constexpr gpio_num_t STATUS_NEOPIXEL_PIN = GPIO_NUM_0;
constexpr gpio_num_t STATUS_NEOPIXEL_POWER_PIN = GPIO_NUM_2;
constexpr uint32_t MIC_SAMPLE_RATE = 16000;
constexpr uint32_t AUDIO_PLAYBACK_RATE = 8000;

constexpr uint32_t IMU_INTERVAL_MS = 10;       // 100 Hz sensor acquisition
constexpr uint32_t PACKET_INTERVAL_MS = 100;   // ten samples per telemetry packet
constexpr uint32_t ENV_INTERVAL_MS = 1000;
constexpr size_t AUDIO_PLAYBACK_BUFFER_SIZE = AUDIO_PLAYBACK_RATE * PACKET_INTERVAL_MS / 1000;
constexpr size_t BLE_MAX_CHUNK_BYTES = 180;

static const char *DEVICE_NAME = "PAL-V2-Telemetry";
static const char *SERVICE_UUID = "7f510001-5b8d-4a84-9c7c-a07142ab6001";
static const char *DATA_UUID = "7f510002-5b8d-4a84-9c7c-a07142ab6001";
static const char *COMMAND_UUID = "7f510003-5b8d-4a84-9c7c-a07142ab6001";

RTC_DS3231 rtc;
Adafruit_BME680 bme;
Adafruit_LSM6DSOX lsm6dsox;
Adafruit_NeoPixel pixel(1, STATUS_NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
NimBLECharacteristic *dataCharacteristic = nullptr;
i2s_port_t microphonePort = I2S_NUM_0;

struct ImuSample {
  uint32_t offsetMs;
  float ax, ay, az;
  float gx, gy, gz;
};

ImuSample imuSamples[16];
size_t imuCount = 0;
bool rtcReady = false;
bool rtcNeedsSync = false;
bool bmeReady = false;
bool imuReady = false;
bool micReady = false;
volatile bool bleSubscribed = false;
volatile size_t bleChunkBytes = 20;
volatile uint16_t bleConnectionHandle = BLE_HS_CONN_HANDLE_NONE;
uint64_t clockBaseMs = 0;
uint32_t clockBaseMillis = 0;
uint32_t lastImuMs = 0;
uint32_t lastPacketMs = 0;
uint32_t lastEnvMs = 0;
uint32_t lastLedUpdateMs = 0;
uint32_t lastBleActivityMs = 0;
uint32_t bmeReadyAtMs = 0;
bool bmeReading = false;
bool audioStreaming = false;
double audioSquareSum = 0.0;
uint32_t audioSampleCount = 0;
int8_t audioPlaybackSamples[AUDIO_PLAYBACK_BUFFER_SIZE];
size_t audioPlaybackCount = 0;
bool keepPlaybackSample = false;

struct CommandMessage {
  char text[257];
};

QueueHandle_t commandQueue = nullptr;
bool batterySaveMode = false;

String encodeBase64(const uint8_t *data, size_t length) {
  static const char alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  String encoded;
  encoded.reserve(((length + 2) / 3) * 4);
  for (size_t i = 0; i < length; i += 3) {
    const uint32_t a = data[i];
    const uint32_t b = i + 1 < length ? data[i + 1] : 0;
    const uint32_t c = i + 2 < length ? data[i + 2] : 0;
    const uint32_t triple = (a << 16) | (b << 8) | c;
    encoded += alphabet[(triple >> 18) & 0x3F];
    encoded += alphabet[(triple >> 12) & 0x3F];
    encoded += i + 1 < length ? alphabet[(triple >> 6) & 0x3F] : '=';
    encoded += i + 2 < length ? alphabet[triple & 0x3F] : '=';
  }
  return encoded;
}

float readBatteryVoltage() {
  // Adafruit Feather ESP32 V2 uses a 1:2 voltage divider (two 100k resistors) on GPIO35.
  const uint32_t raw = analogRead(PIN_BATTERY);
  const float vAdc = (static_cast<float>(raw) / 4095.0f) * 3.3f;
  const float vBat = vAdc * 2.0f;
  return vBat;
}

uint8_t calculateBatteryPercentage(float vBat) {
  if (vBat >= 4.20f) return 100;
  if (vBat <= 3.30f) return 0;
  return static_cast<uint8_t>(constrain((vBat - 3.30f) / (4.20f - 3.30f) * 100.0f, 0.0f, 100.0f));
}

enum SystemStatusColor {
  COLOR_BLUE,   // BLE Connected
  COLOR_RED,    // Sensor Error or High Gas Alert
  COLOR_AMBER,  // RTC Drift / Low Battery
  COLOR_GREEN   // All Systems Normal
};

SystemStatusColor evaluateSystemStatus(bool bleConn, bool rtcOk, bool rtcSyncNeeded, bool imuOk, bool bmeOk, bool micOk, float vBat, uint32_t gasOhms) {
  if (bleConn) return COLOR_BLUE;
  if (!rtcOk || !imuOk || !bmeOk || !micOk || (bmeOk && gasOhms > 0 && gasOhms < 20000)) return COLOR_RED;
  if (rtcSyncNeeded || (vBat > 0.5f && vBat < 3.48f)) return COLOR_AMBER;
  return COLOR_GREEN;
}

void performSafeBleDisconnect() {
  bleSubscribed = false;
  audioStreaming = false;
  audioPlaybackCount = 0;
  const uint16_t handle = bleConnectionHandle;
  bleConnectionHandle = BLE_HS_CONN_HANDLE_NONE;

  NimBLEServer *server = NimBLEDevice::getServer();
  if (server != nullptr && handle != BLE_HS_CONN_HANDLE_NONE && server->getConnectedCount() > 0) {
    server->disconnect(handle);
  }

  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  if (adv != nullptr && !adv->isAdvertising()) {
    adv->start();
  }
}

void checkBleSafetyTimeout() {
  NimBLEServer *server = NimBLEDevice::getServer();
  if (server == nullptr) return;
  const size_t connCount = server->getConnectedCount();

  if (connCount == 0) {
    if (bleSubscribed || bleConnectionHandle != BLE_HS_CONN_HANDLE_NONE) {
      performSafeBleDisconnect();
    }
    NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
    if (adv != nullptr && !adv->isAdvertising()) {
      adv->start();
    }
    return;
  }

  const uint32_t now = millis();
  if (lastBleActivityMs > 0 && (now - lastBleActivityMs > 15000)) {
    performSafeBleDisconnect();
  }
}

void updateNeoPixelStatus() {
  checkBleSafetyTimeout();
  NimBLEServer *server = NimBLEDevice::getServer();
  const bool activeBleConnected = bleSubscribed && (server != nullptr && server->getConnectedCount() > 0);
  const float vBat = readBatteryVoltage();
  const SystemStatusColor status = evaluateSystemStatus(
    activeBleConnected, rtcReady, rtcNeedsSync, imuReady, bmeReady, micReady, vBat, bme.gas_resistance
  );

  uint8_t r = 0, g = 0, b = 0;
  switch (status) {
    case COLOR_BLUE:
      b = 255;
      break;
    case COLOR_RED:
      r = 255;
      break;
    case COLOR_AMBER:
      r = 255; g = 140;
      break;
    case COLOR_GREEN:
    default:
      g = 255;
      break;
  }

  if (batterySaveMode) {
    const uint32_t phase = millis() % 5000;
    constexpr uint32_t PULSE_DURATION_MS = 600;
    if (phase < PULSE_DURATION_MS) {
      const float factor = sinf((static_cast<float>(phase) / static_cast<float>(PULSE_DURATION_MS)) * M_PI);
      pixel.setPixelColor(0, pixel.Color(
        static_cast<uint8_t>(r * factor),
        static_cast<uint8_t>(g * factor),
        static_cast<uint8_t>(b * factor)
      ));
    } else {
      pixel.setPixelColor(0, pixel.Color(0, 0, 0));
    }
  } else {
    pixel.setPixelColor(0, pixel.Color(r, g, b));
  }
  pixel.show();
}

uint64_t nowUnixMs() {
  return clockBaseMs + static_cast<uint32_t>(millis() - clockBaseMillis);
}

void syncClockBase() {
  DateTime now = rtc.now();
  clockBaseMs = static_cast<uint64_t>(now.unixtime()) * 1000ULL;
  clockBaseMillis = millis();
}

void sendLine(const String &line) {
  Serial.println(line);
  if (dataCharacteristic == nullptr || !bleSubscribed) {
    return;
  }

  const size_t chunkBytes = bleChunkBytes;
  const uint16_t connectionHandle = bleConnectionHandle;
  bool notifySuccess = true;

  for (size_t start = 0; start < line.length(); start += chunkBytes) {
    const size_t length = min(chunkBytes, line.length() - start);
    if (dataCharacteristic->notify(reinterpret_cast<const uint8_t *>(line.c_str() + start), length, connectionHandle)) {
      lastBleActivityMs = millis();
    } else {
      notifySuccess = false;
      break;
    }
  }

  if (notifySuccess) {
    const uint8_t newline = '\n';
    if (dataCharacteristic->notify(&newline, 1, connectionHandle)) {
      lastBleActivityMs = millis();
    } else {
      notifySuccess = false;
    }
  }

  if (!notifySuccess) {
    performSafeBleDisconnect();
  }
}

void sendStatus(const char *state, const char *detail) {
  String message = "{\"type\":\"status\",\"time_ms\":" + String(nowUnixMs()) +
                   ",\"state\":\"" + state + "\",\"detail\":\"" + detail + "\"}";
  sendLine(message);
}

void setRtcFromUnixMs(uint64_t unixMs) {
  if (!rtcReady) {
    sendStatus("command_error", "DS3231 is unavailable; clock was not changed");
    return;
  }
  rtc.adjust(DateTime(static_cast<uint32_t>(unixMs / 1000ULL)));
  clockBaseMs = unixMs;
  clockBaseMillis = millis();
  rtcNeedsSync = false;
  sendStatus("clock_set", "RTC updated from dashboard (UTC)");
}

void handleCommand(String command) {
  command.trim();
  if (command.indexOf("toggle_battery_save") >= 0 || command.indexOf("battery_save") >= 0) {
    batterySaveMode = !batterySaveMode;
    if (batterySaveMode) {
      sendStatus("battery_save_on", "Battery save mode ENABLED (LED pulsating every 5s)");
    } else {
      sendStatus("battery_save_off", "Battery save mode DISABLED (LED continuous status)");
    }
    updateNeoPixelStatus();
    return;
  }
  if (command.indexOf("disconnect_ble") >= 0) {
    performSafeBleDisconnect();
    sendStatus("ble_disconnected", "Bluetooth connection cleared & advertising restarted");
    return;
  }
  if (command.indexOf("start_audio") >= 0) {
    audioStreaming = true;
    audioPlaybackCount = 0;
    sendStatus("recording", "PCM audio stream started");
    return;
  }
  if (command.indexOf("stop_audio") >= 0) {
    audioStreaming = false;
    audioPlaybackCount = 0;
    sendStatus("recording_stopped", "PCM audio stream stopped");
    return;
  }
  if (command.indexOf("set_time") < 0) {
    sendStatus("command_error", "Unknown command");
    return;
  }
  const int key = command.indexOf("\"unix_ms\"");
  const int colon = key < 0 ? -1 : command.indexOf(':', key);
  if (colon < 0) {
    sendStatus("command_error", "Missing unix_ms");
    return;
  }
  const char *number = command.c_str() + colon + 1;
  while (isspace(static_cast<unsigned char>(*number))) ++number;
  errno = 0;
  char *end = nullptr;
  const uint64_t unixMs = strtoull(number, &end, 10);
  while (end != nullptr && isspace(static_cast<unsigned char>(*end))) ++end;
  if (errno == ERANGE || end == number || end == nullptr || (*end != '}' && *end != ',') ||
      unixMs < 1577836800000ULL || unixMs >= 4102444800000ULL) {
    sendStatus("command_error", "Invalid unix_ms");
    return;
  }
  setRtcFromUnixMs(unixMs);
}

class CommandCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *characteristic, NimBLEConnInfo & /*connInfo*/) override {
    lastBleActivityMs = millis();
    if (commandQueue == nullptr) return;
    const NimBLEAttValue value = characteristic->getValue();
    CommandMessage message = {};
    if (value.length() > sizeof(message.text) - 1) {
      strncpy(message.text, "command_too_long", sizeof(message.text) - 1);
    } else {
      memcpy(message.text, value.data(), value.length());
    }
    xQueueSend(commandQueue, &message, 0);
  }
};

class DataCallbacks : public NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic * /*characteristic*/, NimBLEConnInfo &connInfo, uint16_t subValue) override {
    lastBleActivityMs = millis();
    bleSubscribed = subValue != 0;
    if (bleSubscribed) {
      bleConnectionHandle = connInfo.getConnHandle();
      bleChunkBytes = min(BLE_MAX_CHUNK_BYTES, static_cast<size_t>(max(23, static_cast<int>(connInfo.getMTU())) - 3));
    }
  }
};

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer * /*server*/, NimBLEConnInfo &connInfo) override {
    lastBleActivityMs = millis();
    bleConnectionHandle = connInfo.getConnHandle();
  }
  void onDisconnect(NimBLEServer * /*server*/, NimBLEConnInfo & /*connInfo*/, int /*reason*/) override {
    performSafeBleDisconnect();
  }
};

CommandCallbacks commandCallbacks;
DataCallbacks dataCallbacks;
ServerCallbacks serverCallbacks;

void beginBle() {
  NimBLEDevice::init(DEVICE_NAME);
  NimBLEDevice::setMTU(247);
  NimBLEServer *server = NimBLEDevice::createServer();
  server->setCallbacks(&serverCallbacks);
  server->advertiseOnDisconnect(true);
  NimBLEService *service = server->createService(SERVICE_UUID);
  dataCharacteristic = service->createCharacteristic(DATA_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  NimBLECharacteristic *commandCharacteristic = service->createCharacteristic(
    COMMAND_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
  );
  dataCharacteristic->setCallbacks(&dataCallbacks);
  commandCharacteristic->setCallbacks(&commandCallbacks);
  dataCharacteristic->setValue("PAL V2 ready");
  server->start();

  NimBLEAdvertising *advertising = NimBLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setName(DEVICE_NAME);
  advertising->enableScanResponse(true);
  advertising->start();
}

void beginMicrophone() {
  i2s_config_t config = {};
  config.mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_RX);
  config.sample_rate = MIC_SAMPLE_RATE;
  config.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
  config.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  config.communication_format = static_cast<i2s_comm_format_t>(I2S_COMM_FORMAT_STAND_I2S);
  config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  config.dma_buf_count = 8;
  config.dma_buf_len = 128;
  config.use_apll = false;

  i2s_pin_config_t pins = {};
  pins.bck_io_num = MIC_BCLK;
  pins.ws_io_num = MIC_LRCL;
  pins.data_out_num = I2S_PIN_NO_CHANGE;
  pins.data_in_num = MIC_DOUT;

  micReady = i2s_driver_install(microphonePort, &config, 0, nullptr) == ESP_OK &&
             i2s_set_pin(microphonePort, &pins) == ESP_OK &&
             i2s_start(microphonePort) == ESP_OK;
  if (!micReady) {
    i2s_driver_uninstall(microphonePort);
  }
}

void readMicrophone() {
  if (!micReady) return;
  int32_t frames[128];
  size_t bytesRead = 0;
  if (i2s_read(microphonePort, frames, sizeof(frames), &bytesRead, 0) != ESP_OK) return;
  const size_t words = bytesRead / sizeof(int32_t);
  for (size_t i = 0; i + 1 < words; i += 2) {
    // Accumulate normalized PCM energy for a 100 ms RMS dBFS reading.
    // ICS43434 drives only its SEL-selected slot; choose whichever slot is active.
    const int64_t leftMagnitude = frames[i] < 0 ? -static_cast<int64_t>(frames[i]) : frames[i];
    const int64_t rightMagnitude = frames[i + 1] < 0 ? -static_cast<int64_t>(frames[i + 1]) : frames[i + 1];
    const int32_t raw = leftMagnitude >= rightMagnitude ? frames[i] : frames[i + 1];
    const double normalized = static_cast<double>(raw) / 2147483648.0;
    audioSquareSum += normalized * normalized;
    ++audioSampleCount;
    keepPlaybackSample = !keepPlaybackSample;
    if (audioStreaming && keepPlaybackSample && audioPlaybackCount < AUDIO_PLAYBACK_BUFFER_SIZE) {
      // Downsample to 8 kHz and quantize the left-justified microphone PCM to
      // signed 8-bit audio for bandwidth-efficient browser playback.
      audioPlaybackSamples[audioPlaybackCount++] = static_cast<int8_t>(constrain(raw >> 24, -128, 127));
    }
  }
}

void collectImu() {
  if (!imuReady || imuCount == sizeof(imuSamples) / sizeof(imuSamples[0])) return;
  sensors_event_t accel, gyro, temp;
  lsm6dsox.getEvent(&accel, &gyro, &temp);
  ImuSample &sample = imuSamples[imuCount++];
  sample.offsetMs = millis() - lastPacketMs;
  sample.ax = accel.acceleration.x;
  sample.ay = accel.acceleration.y;
  sample.az = accel.acceleration.z;
  sample.gx = gyro.gyro.x;
  sample.gy = gyro.gyro.y;
  sample.gz = gyro.gyro.z;
}

void sendTelemetryPacket() {
  const uint32_t elapsedMs = millis() - lastPacketMs;
  const uint64_t audioTime = nowUnixMs();
  const uint64_t packetTime = audioTime - elapsedMs;
  String packet = "{\"type\":\"telemetry\",\"time_ms\":" + String(packetTime) + ",\"imu\":[";
  packet.reserve(audioStreaming ? 2400 : 1200);
  for (size_t i = 0; i < imuCount; ++i) {
    const ImuSample &s = imuSamples[i];
    if (i) packet += ',';
    packet += "[" + String(s.offsetMs) + ',' + String(s.ax, 4) + ',' + String(s.ay, 4) + ',' +
              String(s.az, 4) + ',' + String(s.gx, 4) + ',' + String(s.gy, 4) + ',' + String(s.gz, 4) + "]";
  }
  // dBFS is relative to the digital microphone's full-scale PCM value. A
  // calibrated dB SPL value would additionally require an acoustic calibrator.
  constexpr double DBFS_FLOOR = -120.0;
  double audioDbfs = DBFS_FLOOR;
  if (audioSampleCount > 0 && audioSquareSum > 0.0) {
    const double rms = sqrt(audioSquareSum / audioSampleCount);
    audioDbfs = max(DBFS_FLOOR, 20.0 * log10(rms));
  }
  packet += "],\"audio_time_ms\":" + String(audioTime) +
            ",\"audio_samples\":" + String(audioSampleCount) +
            ",\"audio_dbfs\":" + String(audioDbfs, 2);
  if (audioStreaming) {
    packet += ",\"audio_rate_hz\":" + String(AUDIO_PLAYBACK_RATE) +
              ",\"audio_pcm_s8_b64\":\"" +
              encodeBase64(reinterpret_cast<const uint8_t *>(audioPlaybackSamples), audioPlaybackCount) + "\"";
  }
  packet += '}';
  sendLine(packet);
  imuCount = 0;
  audioSquareSum = 0.0;
  audioSampleCount = 0;
  audioPlaybackCount = 0;
}

void sendEnvironment() {
  if (!bme.endReading()) {
    sendStatus("sensor_error", "BME688 read failed");
    return;
  }
  const float vBat = readBatteryVoltage();
  const uint8_t pctBat = calculateBatteryPercentage(vBat);
  String packet = "{\"type\":\"environment\",\"time_ms\":" + String(nowUnixMs()) +
                  ",\"temperature_c\":" + String(bme.temperature, 2) +
                  ",\"humidity_pct\":" + String(bme.humidity, 2) +
                  ",\"pressure_hpa\":" + String(bme.pressure / 100.0F, 2) +
                  ",\"gas_ohms\":" + String(bme.gas_resistance) +
                  ",\"battery_v\":" + String(vBat, 2) +
                  ",\"battery_pct\":" + String(pctBat) + "}";
  sendLine(packet);
}

void startEnvironmentReading() {
  if (!bmeReady || bmeReading) return;
  const uint32_t readyAt = bme.beginReading();
  if (readyAt == 0) {
    sendStatus("sensor_error", "BME688 start failed");
    return;
  }
  bmeReadyAtMs = readyAt;
  bmeReading = true;
}

void readSerialCommands() {
  static String input;
  static bool discarding = false;
  while (Serial.available()) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      if (discarding) {
        sendStatus("command_error", "Serial command exceeded 256 bytes");
      } else if (!input.isEmpty()) {
        handleCommand(input);
      }
      input = "";
      discarding = false;
    } else if (input.length() < 256) {
      input += c;
    } else {
      discarding = true;
    }
  }
}

void readBleCommands() {
  if (commandQueue == nullptr) return;
  CommandMessage message;
  while (xQueueReceive(commandQueue, &message, 0) == pdTRUE) {
    if (strcmp(message.text, "command_too_long") == 0) {
      sendStatus("command_error", "BLE command exceeded 256 bytes");
    } else {
      handleCommand(String(message.text));
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  pinMode(STATUS_NEOPIXEL_POWER_PIN, OUTPUT);
  digitalWrite(STATUS_NEOPIXEL_POWER_PIN, HIGH);
  pixel.begin();
  pixel.setBrightness(20);

  // Onboard NeoPixel RGB Boot Self-Test (Red -> Green -> Blue -> Yellow -> Cyan -> Magenta -> White -> Off)
  const uint32_t bootColors[] = {
    pixel.Color(255, 0, 0),     // Red
    pixel.Color(0, 255, 0),     // Green
    pixel.Color(0, 0, 255),     // Blue
    pixel.Color(255, 255, 0),   // Yellow
    pixel.Color(0, 255, 255),   // Cyan
    pixel.Color(255, 0, 255),   // Magenta
    pixel.Color(255, 255, 255), // White
    pixel.Color(0, 0, 0)        // Off
  };
  for (size_t i = 0; i < sizeof(bootColors) / sizeof(bootColors[0]); ++i) {
    pixel.setPixelColor(0, bootColors[i]);
    pixel.show();
    delay(150);
  }

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  Wire.begin(); // STEMMA QT uses SDA=GPIO22 and SCL=GPIO20 on this Feather.
  commandQueue = xQueueCreate(4, sizeof(CommandMessage));

  rtcReady = rtc.begin();
  if (rtcReady) {
    rtcNeedsSync = rtc.lostPower();
    if (rtcNeedsSync) rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    syncClockBase();
  } else {
    clockBaseMs = 0;
    clockBaseMillis = millis();
  }

  bmeReady = bme.begin(0x77) || bme.begin(0x76);
  if (bmeReady) {
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150);
  }

  imuReady = lsm6dsox.begin_I2C();
  if (imuReady) {
    lsm6dsox.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
    lsm6dsox.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
    lsm6dsox.setAccelDataRate(LSM6DS_RATE_104_HZ);
    lsm6dsox.setGyroDataRate(LSM6DS_RATE_104_HZ);
  }

  beginMicrophone();
  beginBle();
  lastPacketMs = millis();
  lastImuMs = lastPacketMs;
  lastEnvMs = millis();
  startEnvironmentReading();
  sendStatus("ready", "JSON Lines over USB serial and BLE notifications");
  if (!rtcReady) sendStatus("sensor_error", "DS3231 not found");
  if (rtcNeedsSync) sendStatus("clock_warning", "RTC lost power; synchronize it from the dashboard");
  if (!imuReady) sendStatus("sensor_error", "LSM6DSOX not found");
  if (!bmeReady) sendStatus("sensor_error", "BME688 not found");
  if (!micReady) sendStatus("sensor_error", "ICS43434 I2S setup failed");
}

void loop() {
  readBleCommands();
  readSerialCommands();
  readMicrophone();
  const uint32_t now = millis();
  if (now - lastImuMs >= IMU_INTERVAL_MS) {
    lastImuMs = now;
    collectImu();
  }
  if (now - lastPacketMs >= PACKET_INTERVAL_MS) {
    sendTelemetryPacket();
    lastPacketMs = now;
  }
  if (bmeReading && static_cast<int32_t>(now - bmeReadyAtMs) >= 0) {
    sendEnvironment();
    bmeReading = false;
  }
  if (now - lastEnvMs >= ENV_INTERVAL_MS) {
    lastEnvMs = now;
    startEnvironmentReading();
  }
  if (now - lastLedUpdateMs >= 250) {
    lastLedUpdateMs = now;
    updateNeoPixelStatus();
  }
}