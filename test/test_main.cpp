#include <unity.h>
#include <string>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include "ima_adpcm.h"
#include "audio_spectrum.h"

static inline int32_t constrainVal(int32_t val, int32_t minVal, int32_t maxVal) {
  return val < minVal ? minVal : (val > maxVal ? maxVal : val);
}

std::string encodeBase64Test(const uint8_t *data, size_t length) {
  static const char alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string encoded;
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

int8_t quantizePcm32To8(int32_t raw) {
  return static_cast<int8_t>(constrainVal(raw >> 24, -128, 127));
}

double calculateDbfs(double squareSum, uint32_t count) {
  constexpr double DBFS_FLOOR = -120.0;
  if (count == 0 || squareSum <= 0.0) return DBFS_FLOOR;
  const double rms = sqrt(squareSum / count);
  return std::max(DBFS_FLOOR, 20.0 * log10(rms));
}

void test_base64_encoding() {
  const uint8_t input[] = "PAL-V2";
  std::string result = encodeBase64Test(input, 6);
  TEST_ASSERT_EQUAL_STRING("UEFMLVYy", result.c_str());
}

void test_pcm_quantization() {
  // Max positive 32-bit sample should map to 127
  TEST_ASSERT_EQUAL_INT8(127, quantizePcm32To8(2147483647));
  // Zero sample should map to 0
  TEST_ASSERT_EQUAL_INT8(0, quantizePcm32To8(0));
  // Mid positive sample (0.5 full scale = 1073741824) should map to 64
  TEST_ASSERT_EQUAL_INT8(64, quantizePcm32To8(1073741824));
  // Negative full scale (-2147483648) should map to -128
  TEST_ASSERT_EQUAL_INT8(-128, quantizePcm32To8(-2147483648LL));
}

void test_dbfs_calculation() {
  // Full scale sine/square wave (rms = 1.0) -> 0.0 dBFS
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, static_cast<float>(calculateDbfs(1.0, 1)));
  // Half amplitude (rms = 0.5) -> -6.02 dBFS
  TEST_ASSERT_FLOAT_WITHIN(0.1f, -6.02f, static_cast<float>(calculateDbfs(0.25, 1)));
  // Zero amplitude -> -120.0 dBFS floor
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -120.0f, static_cast<float>(calculateDbfs(0.0, 0)));
}

void test_audio_spectrum_silence_and_near_silence() {
  int16_t samples[pal::AUDIO_SPECTRUM_FFT_SIZE] = {};
  float bands[pal::AUDIO_SPECTRUM_BAND_COUNT];

  pal::calculateAudioSpectrumDbfs(samples, bands);
  for (size_t i = 0; i < pal::AUDIO_SPECTRUM_BAND_COUNT; ++i) {
    TEST_ASSERT_TRUE(std::isfinite(bands[i]));
    TEST_ASSERT_EQUAL_FLOAT(pal::AUDIO_SPECTRUM_FLOOR_DBFS, bands[i]);
  }

  // A single one-LSB impulse is below the configured spectral floor.
  samples[pal::AUDIO_SPECTRUM_FFT_SIZE / 2] = 1;
  pal::calculateAudioSpectrumDbfs(samples, bands);
  for (size_t i = 0; i < pal::AUDIO_SPECTRUM_BAND_COUNT; ++i) {
    TEST_ASSERT_TRUE(std::isfinite(bands[i]));
    TEST_ASSERT_EQUAL_FLOAT(pal::AUDIO_SPECTRUM_FLOOR_DBFS, bands[i]);
  }
}

void test_audio_spectrum_sine_peak_band() {
  constexpr uint32_t sampleRateHz = 16000;
  constexpr float frequencyHz = 1000.0f;
  int16_t samples[pal::AUDIO_SPECTRUM_FFT_SIZE];
  for (size_t i = 0; i < pal::AUDIO_SPECTRUM_FFT_SIZE; ++i) {
    samples[i] = static_cast<int16_t>(
        16384.0f * sinf(2.0f * 3.14159265358979323846f * frequencyHz *
                        static_cast<float>(i) / static_cast<float>(sampleRateHz)));
  }

  float bands[pal::AUDIO_SPECTRUM_BAND_COUNT];
  pal::calculateAudioSpectrumDbfs(samples, bands);
  size_t strongestBand = 0;
  for (size_t i = 1; i < pal::AUDIO_SPECTRUM_BAND_COUNT; ++i) {
    if (bands[i] > bands[strongestBand]) strongestBand = i;
  }

  // Linear 250 Hz-wide bands place 1000 Hz in zero-based band 4.
  TEST_ASSERT_EQUAL_UINT32(4, strongestBand);
  TEST_ASSERT_TRUE(bands[strongestBand] <= pal::AUDIO_SPECTRUM_CEILING_DBFS);
  TEST_ASSERT_TRUE(bands[strongestBand] >= pal::AUDIO_SPECTRUM_FLOOR_DBFS);
}

void test_audio_spectrum_packet_schema_and_framing() {
  float bands[pal::AUDIO_SPECTRUM_BAND_COUNT];
  for (size_t i = 0; i < pal::AUDIO_SPECTRUM_BAND_COUNT; ++i) {
    bands[i] = -100.0f + static_cast<float>(i) * 3.5f;
  }
  bands[0] = NAN;
  bands[1] = INFINITY;

  char packet[512];
  TEST_ASSERT_TRUE(pal::formatAudioSpectrumPacket(
      packet, sizeof(packet), 123456789ULL, 16000, bands));
  TEST_ASSERT_NOT_NULL(strstr(packet, "\"type\":\"audio_spectrum\""));
  TEST_ASSERT_NOT_NULL(strstr(packet, "\"time_ms\":123456789"));
  TEST_ASSERT_NOT_NULL(strstr(packet, "\"sample_rate_hz\":16000"));
  TEST_ASSERT_NOT_NULL(strstr(packet, "\"fft_size\":256"));
  TEST_ASSERT_NOT_NULL(strstr(packet, "\"min_hz\":0"));
  TEST_ASSERT_NOT_NULL(strstr(packet, "\"max_hz\":8000"));
  TEST_ASSERT_NOT_NULL(strstr(packet, "\"scale\":\"linear\""));
  TEST_ASSERT_NULL(strstr(packet, "nan"));
  TEST_ASSERT_NULL(strstr(packet, "inf"));

  // Parse the numeric array and require exactly 32 bounded JSON numbers.
  const char *cursor = strstr(packet, "\"audio_fft\":[");
  TEST_ASSERT_NOT_NULL(cursor);
  cursor += strlen("\"audio_fft\":[");
  for (size_t i = 0; i < pal::AUDIO_SPECTRUM_BAND_COUNT; ++i) {
    char *end = nullptr;
    const float value = strtof(cursor, &end);
    TEST_ASSERT_TRUE(end != cursor);
    TEST_ASSERT_TRUE(std::isfinite(value));
    TEST_ASSERT_TRUE(value >= pal::AUDIO_SPECTRUM_FLOOR_DBFS);
    TEST_ASSERT_TRUE(value <= pal::AUDIO_SPECTRUM_CEILING_DBFS);
    TEST_ASSERT_EQUAL_CHAR(i + 1 == pal::AUDIO_SPECTRUM_BAND_COUNT ? ']' : ',', *end);
    cursor = end + 1;
  }
  TEST_ASSERT_EQUAL_CHAR('}', *cursor);
  TEST_ASSERT_EQUAL_CHAR('\0', *(cursor + 1));

  const std::string framed = std::string(packet) + '\n';
  TEST_ASSERT_EQUAL_CHAR('\n', framed.back());
  TEST_ASSERT_EQUAL_UINT32(strlen(packet) + 1, framed.size());
}

void test_audio_spectrum_rate_limiter() {
  TEST_ASSERT_FALSE(pal::audioSpectrumPacketDue(199, 0, 200, true));
  TEST_ASSERT_TRUE(pal::audioSpectrumPacketDue(200, 0, 200, true));
  TEST_ASSERT_FALSE(pal::audioSpectrumPacketDue(400, 200, 200, false));
  TEST_ASSERT_TRUE(pal::audioSpectrumPacketDue(400, 200, 200, true));
  TEST_ASSERT_TRUE(pal::audioSpectrumPacketDue(100, UINT32_MAX - 149, 200, true));
}


uint8_t calculateBatteryPercentageTest(float vBat) {
  if (vBat >= 4.20f) return 100;
  if (vBat <= 3.30f) return 0;
  float pct = (vBat - 3.30f) / (4.20f - 3.30f) * 100.0f;
  return static_cast<uint8_t>(pct < 0.0f ? 0.0f : (pct > 100.0f ? 100.0f : pct));
}

void test_battery_percentage() {
  TEST_ASSERT_EQUAL_UINT8(100, calculateBatteryPercentageTest(4.20f));
  TEST_ASSERT_EQUAL_UINT8(100, calculateBatteryPercentageTest(4.35f));
  TEST_ASSERT_EQUAL_UINT8(50, calculateBatteryPercentageTest(3.75f));
  TEST_ASSERT_EQUAL_UINT8(0, calculateBatteryPercentageTest(3.30f));
  TEST_ASSERT_EQUAL_UINT8(0, calculateBatteryPercentageTest(2.80f));
}

enum SystemStatusColorTest {
  COLOR_BLUE,   // BLE Connected
  COLOR_RED,    // Sensor Error or High Gas Alert
  COLOR_AMBER,  // RTC Drift / Low Battery
  COLOR_GREEN   // All Systems Normal
};

SystemStatusColorTest evaluateSystemStatusTest(bool bleConn, bool rtcOk, bool rtcSyncNeeded, bool imuOk, bool bmeOk, bool micOk, float vBat, uint32_t gasOhms) {
  if (bleConn) return COLOR_BLUE;
  if (!rtcOk || !imuOk || !bmeOk || !micOk || (bmeOk && gasOhms > 0 && gasOhms < 20000)) return COLOR_RED;
  if (rtcSyncNeeded || (vBat > 0.5f && vBat < 3.48f)) return COLOR_AMBER;
  return COLOR_GREEN;
}

void test_neopixel_status_priority() {
  // BLE connected -> BLUE
  TEST_ASSERT_EQUAL_INT(COLOR_BLUE, evaluateSystemStatusTest(true, true, false, true, true, true, 4.1f, 100000));
  // Sensor error -> RED
  TEST_ASSERT_EQUAL_INT(COLOR_RED, evaluateSystemStatusTest(false, false, false, true, true, true, 4.1f, 100000));
  // High gas hazard / drop -> RED
  TEST_ASSERT_EQUAL_INT(COLOR_RED, evaluateSystemStatusTest(false, true, false, true, true, true, 4.1f, 15000));
  // RTC sync needed -> AMBER
  TEST_ASSERT_EQUAL_INT(COLOR_AMBER, evaluateSystemStatusTest(false, true, true, true, true, true, 4.1f, 100000));
  // Low battery -> AMBER
  TEST_ASSERT_EQUAL_INT(COLOR_AMBER, evaluateSystemStatusTest(false, true, false, true, true, true, 3.4f, 100000));
  // All normal -> GREEN
  TEST_ASSERT_EQUAL_INT(COLOR_GREEN, evaluateSystemStatusTest(false, true, false, true, true, true, 4.0f, 100000));
}

float computePulseFactorTest(uint32_t ms) {
  const uint32_t phase = ms % 5000;
  constexpr uint32_t PULSE_DURATION_MS = 600;
  if (phase < PULSE_DURATION_MS) {
    return sinf((static_cast<float>(phase) / static_cast<float>(PULSE_DURATION_MS)) * 3.14159265f);
  }
  return 0.0f;
}

void test_battery_save_pulse_math() {
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 1.0f, computePulseFactorTest(300));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, computePulseFactorTest(1000));
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 1.0f, computePulseFactorTest(5300));
}

struct UnpackedImuWordTest {
  uint8_t tag;
  int16_t rawX, rawY, rawZ;
  float axG, ayG, azG;
  float gxRads, gyRads, gzRads;
};

UnpackedImuWordTest unpackImuFifoWordTest(const uint8_t rawBytes[7]) {
  UnpackedImuWordTest result = {};
  result.tag = rawBytes[0] >> 3;
  result.rawX = static_cast<int16_t>(rawBytes[1] | (rawBytes[2] << 8));
  result.rawY = static_cast<int16_t>(rawBytes[3] | (rawBytes[4] << 8));
  result.rawZ = static_cast<int16_t>(rawBytes[5] | (rawBytes[6] << 8));

  if (result.tag == 0x02) { // Accel tag
    constexpr float accelScaleG = 0.000122f; // 0.122 mg/LSB for 4G range
    result.axG = static_cast<float>(result.rawX) * accelScaleG;
    result.ayG = static_cast<float>(result.rawY) * accelScaleG;
    result.azG = static_cast<float>(result.rawZ) * accelScaleG;
  } else if (result.tag == 0x01) { // Gyro tag
    constexpr float gyroScaleRads = 17.50f * (3.14159265f / 180.0f) / 1000.0f; // 17.50 mdps/LSB for 500 dps range
    result.gxRads = static_cast<float>(result.rawX) * gyroScaleRads;
    result.gyRads = static_cast<float>(result.rawY) * gyroScaleRads;
    result.gzRads = static_cast<float>(result.rawZ) * gyroScaleRads;
  }
  return result;
}

void test_imu_fifo_unpacking() {
  // Test Accel FIFO word tag (0x02 << 3 = 0x10) with raw 1.0g (8196 LSB) on X axis
  const uint8_t accelWord[7] = { 0x10, 0x04, 0x20, 0x00, 0x00, 0xFC, 0xDF };
  UnpackedImuWordTest accelRes = unpackImuFifoWordTest(accelWord);

  TEST_ASSERT_EQUAL_UINT8(0x02, accelRes.tag);
  TEST_ASSERT_EQUAL_INT16(8196, accelRes.rawX);
  TEST_ASSERT_EQUAL_INT16(0, accelRes.rawY);
  TEST_ASSERT_EQUAL_INT16(-8196, accelRes.rawZ);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.00f, accelRes.axG);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -1.00f, accelRes.azG);

  // Test Gyro FIFO word tag (0x01 << 3 = 0x08) with raw 1000 LSB on X axis
  const uint8_t gyroWord[7] = { 0x08, 0xE8, 0x03, 0x00, 0x00, 0x00, 0x00 };
  UnpackedImuWordTest gyroRes = unpackImuFifoWordTest(gyroWord);

  TEST_ASSERT_EQUAL_UINT8(0x01, gyroRes.tag);
  TEST_ASSERT_EQUAL_INT16(1000, gyroRes.rawX);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.3054f, gyroRes.gxRads);
}

void test_adpcm_encoder_decoder_roundtrip() {
  constexpr size_t numSamples = 160; // 20 ms at 8 kHz
  int16_t originalPcm[numSamples];
  for (size_t i = 0; i < numSamples; ++i) {
    // Generate 440 Hz sine wave with 20000 peak amplitude
    originalPcm[i] = static_cast<int16_t>(20000.0f * sinf(2.0f * 3.14159265f * 440.0f * i / 8000.0f));
  }

  ImaAdpcmEncoder encoder;
  ImaAdpcmDecoder decoder;

  TEST_ASSERT_EQUAL_INT(0, encoder.getStepIndex());
  TEST_ASSERT_EQUAL_INT16(0, encoder.getPredictedSample());

  uint8_t adpcmOutput[numSamples / 2];
  const size_t encodedBytes = encoder.encodeBlock(originalPcm, numSamples, adpcmOutput);
  TEST_ASSERT_EQUAL_UINT32(numSamples / 2, encodedBytes);

  // Predictor step size should adapt upwards from 0 during strong signal variations
  TEST_ASSERT_TRUE(encoder.getStepIndex() > 0);

  int16_t decodedPcm[numSamples];
  const size_t decodedSamples = decoder.decodeBlock(adpcmOutput, encodedBytes, decodedPcm);
  TEST_ASSERT_EQUAL_UINT32(numSamples, decodedSamples);

  // Encoder and Decoder predictor state must stay perfectly synchronized
  TEST_ASSERT_EQUAL_INT16(encoder.getPredictedSample(), decoder.getPredictedSample());
  TEST_ASSERT_EQUAL_INT(encoder.getStepIndex(), decoder.getStepIndex());

  // Calculate Signal-to-Noise Ratio (SNR) in dB (skipping initial 10-sample predictor ramp-up)
  double signalPower = 0.0;
  double noisePower = 0.0;
  for (size_t i = 10; i < numSamples; ++i) {
    const double orig = originalPcm[i];
    const double diff = orig - decodedPcm[i];
    signalPower += orig * orig;
    noisePower += diff * diff;
  }

  TEST_ASSERT_TRUE(signalPower > 0.0);
  TEST_ASSERT_TRUE(noisePower > 0.0);

  const double snrDb = 10.0 * log10(signalPower / noisePower);
  // 4-bit IMA-ADPCM achieves >15 dB SNR fidelity during steady-state tracking
  TEST_ASSERT_TRUE(snrDb > 15.0);
}

struct IaqResultTest {
  float iaq;
  float vocPpm;
  float eco2Ppm;
};

IaqResultTest calculateIaqScoreTest(float tempC, float humPct, uint32_t gasOhms) {
  if (gasOhms == 0) {
    return {0.0f, 0.0f, 400.0f};
  }

  // 1. Humidity Contribution (25% weight)
  float sHum = 0.0f;
  if (humPct >= 38.0f && humPct <= 42.0f) {
    sHum = 25.0f;
  } else if (humPct < 38.0f) {
    sHum = (0.25f / 40.0f) * humPct * 100.0f;
  } else {
    sHum = ((-0.25f / 60.0f) * humPct + 0.4166f) * 100.0f;
  }

  // 2. Gas Resistance Contribution (75% weight)
  float rGas = static_cast<float>(gasOhms);
  constexpr float R_UPPER = 50000.0f; // Good air
  constexpr float R_LOWER = 5000.0f;  // Bad air
  if (rGas > R_UPPER) rGas = R_UPPER;
  if (rGas < R_LOWER) rGas = R_LOWER;

  float sGas = ((0.75f / (R_UPPER - R_LOWER)) * rGas - (R_LOWER * (0.75f / (R_UPPER - R_LOWER)))) * 100.0f;

  // 3. IAQ Score and VOC estimation
  float iaqPct = sHum + sGas;
  float rawIaq = (100.0f - iaqPct) * 5.0f;
  if (rawIaq < 0.0f) rawIaq = 0.0f;
  if (rawIaq > 500.0f) rawIaq = 500.0f;

  const float vocPpm = (rawIaq / 500.0f) * 10.0f;
  const float eco2Ppm = 400.0f + (rawIaq * 8.0f);

  return {rawIaq, vocPpm, eco2Ppm};
}

void test_iaq_calculation() {
  // Test 1: Clean baseline air at 40% RH, 50,000 Ω -> IAQ 0, VOC 0.0 ppm, eCO2 400 ppm
  IaqResultTest resClean = calculateIaqScoreTest(25.0f, 40.0f, 50000);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, resClean.iaq);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.00f, resClean.vocPpm);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 400.0f, resClean.eco2Ppm);

  // Test 2: Bad air at 10% RH, 5,000 Ω -> IAQ ~468
  IaqResultTest resBad = calculateIaqScoreTest(25.0f, 10.0f, 5000);
  TEST_ASSERT_FLOAT_WITHIN(2.0f, 468.75f, resBad.iaq);
  TEST_ASSERT_TRUE(resBad.vocPpm > 8.0f);
  TEST_ASSERT_TRUE(resBad.eco2Ppm > 4000.0f);

  // Test 3: Moderate air quality (IAQ 100) -> 40% RH, 38,000 Ω
  IaqResultTest resMod = calculateIaqScoreTest(25.0f, 40.0f, 38000);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 100.0f, resMod.iaq);

  // Test 4: Poor air quality (IAQ 200) -> 40% RH, 26,000 Ω
  IaqResultTest resPoor = calculateIaqScoreTest(25.0f, 40.0f, 26000);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 200.0f, resPoor.iaq);

  // Test 5: Hazardous air quality (IAQ ~416) -> 80% RH, 10,000 Ω
  IaqResultTest resHazard = calculateIaqScoreTest(25.0f, 80.0f, 10000);
  TEST_ASSERT_FLOAT_WITHIN(2.0f, 416.7f, resHazard.iaq);
}


int main(int argc, char **argv) {
  UNITY_BEGIN();
  RUN_TEST(test_base64_encoding);
  RUN_TEST(test_pcm_quantization);
  RUN_TEST(test_dbfs_calculation);
  RUN_TEST(test_audio_spectrum_silence_and_near_silence);
  RUN_TEST(test_audio_spectrum_sine_peak_band);
  RUN_TEST(test_audio_spectrum_packet_schema_and_framing);
  RUN_TEST(test_audio_spectrum_rate_limiter);
  RUN_TEST(test_battery_percentage);
  RUN_TEST(test_neopixel_status_priority);
  RUN_TEST(test_battery_save_pulse_math);
  RUN_TEST(test_imu_fifo_unpacking);
  RUN_TEST(test_adpcm_encoder_decoder_roundtrip);
  RUN_TEST(test_iaq_calculation);
  return UNITY_END();
}


