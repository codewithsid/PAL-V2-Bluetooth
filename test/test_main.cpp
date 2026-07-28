#include <unity.h>
#include <string>
#include <algorithm>
#include <cmath>
#include <cstdint>

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

int main(int argc, char **argv) {
  UNITY_BEGIN();
  RUN_TEST(test_base64_encoding);
  RUN_TEST(test_pcm_quantization);
  RUN_TEST(test_dbfs_calculation);
  RUN_TEST(test_battery_percentage);
  RUN_TEST(test_neopixel_status_priority);
  RUN_TEST(test_battery_save_pulse_math);
  return UNITY_END();
}
