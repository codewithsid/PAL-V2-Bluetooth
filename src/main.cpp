#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>
#include <Adafruit_BME680.h>
#include <Adafruit_LSM6DSOX.h>
#include <NimBLEDevice.h>
#include <driver/i2s_std.h>
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
NimBLECharacteristic *dataCharacteristic = nullptr;
i2s_chan_handle_t microphoneChannel = nullptr;

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

  // JSON lines are split only at the BLE transport boundary. The dashboard
  // reassembles them by their trailing newline, so larger IMU packets are safe.
  const size_t chunkBytes = bleChunkBytes;
  const uint16_t connectionHandle = bleConnectionHandle;
  for (size_t start = 0; start < line.length(); start += chunkBytes) {
    const size_t length = min(chunkBytes, line.length() - start);
    dataCharacteristic->notify(reinterpret_cast<const uint8_t *>(line.c_str() + start), length, connectionHandle);
  }
  const uint8_t newline = '\n';
  dataCharacteristic->notify(&newline, 1, connectionHandle);
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
  if (command.indexOf("\"cmd\":\"start_audio\"") >= 0) {
    audioStreaming = true;
    audioPlaybackCount = 0;
    sendStatus("recording", "PCM audio stream started");
    return;
  }
  if (command.indexOf("\"cmd\":\"stop_audio\"") >= 0) {
    audioStreaming = false;
    audioPlaybackCount = 0;
    sendStatus("recording_stopped", "PCM audio stream stopped");
    return;
  }
  if (command.indexOf("\"cmd\":\"set_time\"") < 0) {
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
    bleSubscribed = subValue != 0;
    if (bleSubscribed) {
      bleConnectionHandle = connInfo.getConnHandle();
      bleChunkBytes = min(BLE_MAX_CHUNK_BYTES, static_cast<size_t>(max(23, static_cast<int>(connInfo.getMTU())) - 3));
    }
  }
};

class ServerCallbacks : public NimBLEServerCallbacks {
  void onDisconnect(NimBLEServer * /*server*/, NimBLEConnInfo & /*connInfo*/, int /*reason*/) override {
    bleSubscribed = false;
    bleChunkBytes = 20;
    bleConnectionHandle = BLE_HS_CONN_HANDLE_NONE;
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
  NimBLECharacteristic *commandCharacteristic = service->createCharacteristic(COMMAND_UUID, NIMBLE_PROPERTY::WRITE);
  dataCharacteristic->setCallbacks(&dataCallbacks);
  commandCharacteristic->setCallbacks(&commandCallbacks);
  dataCharacteristic->setValue("PAL V2 ready");
  NimBLEAdvertising *advertising = NimBLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setName(DEVICE_NAME);
  advertising->start();
}

void beginMicrophone() {
  i2s_chan_config_t channelConfig = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  channelConfig.dma_desc_num = 8;
  channelConfig.dma_frame_num = 128;
  if (i2s_new_channel(&channelConfig, nullptr, &microphoneChannel) != ESP_OK) return;

  i2s_std_config_t standardConfig = {};
  standardConfig.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(MIC_SAMPLE_RATE);
  standardConfig.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO);
  standardConfig.slot_cfg.slot_mask = I2S_STD_SLOT_BOTH;
  standardConfig.gpio_cfg.mclk = I2S_GPIO_UNUSED;
  standardConfig.gpio_cfg.bclk = MIC_BCLK;
  standardConfig.gpio_cfg.ws = MIC_LRCL;
  standardConfig.gpio_cfg.dout = I2S_GPIO_UNUSED;
  standardConfig.gpio_cfg.din = MIC_DOUT;

  micReady = i2s_channel_init_std_mode(microphoneChannel, &standardConfig) == ESP_OK &&
             i2s_channel_enable(microphoneChannel) == ESP_OK;
  if (!micReady) {
    i2s_del_channel(microphoneChannel);
    microphoneChannel = nullptr;
  }
}

void readMicrophone() {
  if (!micReady) return;
  int32_t frames[128];
  size_t bytesRead = 0;
  if (i2s_channel_read(microphoneChannel, frames, sizeof(frames), &bytesRead, 0) != ESP_OK) return;
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
      audioPlaybackSamples[audioPlaybackCount++] = static_cast<int8_t>(constrain(raw >> 16, -128, 127));
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
  String packet = "{\"type\":\"environment\",\"time_ms\":" + String(nowUnixMs()) +
                  ",\"temperature_c\":" + String(bme.temperature, 2) +
                  ",\"humidity_pct\":" + String(bme.humidity, 2) +
                  ",\"pressure_hpa\":" + String(bme.pressure / 100.0F, 2) +
                  ",\"gas_ohms\":" + String(bme.gas_resistance) + "}";
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
}