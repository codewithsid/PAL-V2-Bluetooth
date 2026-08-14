#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>

namespace pal {

constexpr size_t AUDIO_SPECTRUM_FFT_SIZE = 256;
constexpr size_t AUDIO_SPECTRUM_BAND_COUNT = 32;
constexpr float AUDIO_SPECTRUM_FLOOR_DBFS = -100.0f;
constexpr float AUDIO_SPECTRUM_CEILING_DBFS = 0.0f;

inline bool audioSpectrumPacketDue(uint32_t nowMs, uint32_t lastPacketMs,
                                   uint32_t intervalMs, bool spectrumReady) {
  return spectrumReady && intervalMs > 0 &&
         static_cast<uint32_t>(nowMs - lastPacketMs) >= intervalMs;
}

// Computes a Hann-windowed, single-sided spectrum. Each output band contains
// the RMS amplitude derived from the mean power of the FFT bins in that band.
// Bands have equal linear width from 0 Hz through Nyquist.
inline void calculateAudioSpectrumDbfs(
    const int16_t samples[AUDIO_SPECTRUM_FFT_SIZE],
    float bandsDbfs[AUDIO_SPECTRUM_BAND_COUNT]) {
  constexpr float PI_F = 3.14159265358979323846f;
  float real[AUDIO_SPECTRUM_FFT_SIZE];
  float imag[AUDIO_SPECTRUM_FFT_SIZE] = {};
  float windowSum = 0.0f;

  for (size_t i = 0; i < AUDIO_SPECTRUM_FFT_SIZE; ++i) {
    const float window =
        0.5f - 0.5f * cosf((2.0f * PI_F * static_cast<float>(i)) /
                           static_cast<float>(AUDIO_SPECTRUM_FFT_SIZE - 1));
    real[i] = (static_cast<float>(samples[i]) / 32768.0f) * window;
    windowSum += window;
  }

  // In-place radix-2 Cooley-Tukey FFT.
  for (size_t i = 1, j = 0; i < AUDIO_SPECTRUM_FFT_SIZE; ++i) {
    size_t bit = AUDIO_SPECTRUM_FFT_SIZE >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) {
      const float tempReal = real[i];
      const float tempImag = imag[i];
      real[i] = real[j];
      imag[i] = imag[j];
      real[j] = tempReal;
      imag[j] = tempImag;
    }
  }

  for (size_t length = 2; length <= AUDIO_SPECTRUM_FFT_SIZE; length <<= 1) {
    const float angle = -2.0f * PI_F / static_cast<float>(length);
    const float stepReal = cosf(angle);
    const float stepImag = sinf(angle);
    for (size_t start = 0; start < AUDIO_SPECTRUM_FFT_SIZE; start += length) {
      float twiddleReal = 1.0f;
      float twiddleImag = 0.0f;
      const size_t half = length >> 1;
      for (size_t offset = 0; offset < half; ++offset) {
        const size_t even = start + offset;
        const size_t odd = even + half;
        const float oddReal = real[odd] * twiddleReal - imag[odd] * twiddleImag;
        const float oddImag = real[odd] * twiddleImag + imag[odd] * twiddleReal;
        real[odd] = real[even] - oddReal;
        imag[odd] = imag[even] - oddImag;
        real[even] += oddReal;
        imag[even] += oddImag;

        const float nextReal = twiddleReal * stepReal - twiddleImag * stepImag;
        twiddleImag = twiddleReal * stepImag + twiddleImag * stepReal;
        twiddleReal = nextReal;
      }
    }
  }

  float bandPower[AUDIO_SPECTRUM_BAND_COUNT] = {};
  uint8_t bandBinCount[AUDIO_SPECTRUM_BAND_COUNT] = {};
  constexpr size_t NYQUIST_BIN = AUDIO_SPECTRUM_FFT_SIZE / 2;
  for (size_t bin = 0; bin <= NYQUIST_BIN; ++bin) {
    const size_t band =
        bin == NYQUIST_BIN
            ? AUDIO_SPECTRUM_BAND_COUNT - 1
            : (bin * AUDIO_SPECTRUM_BAND_COUNT) / NYQUIST_BIN;
    const float magnitude = sqrtf(real[bin] * real[bin] + imag[bin] * imag[bin]);
    const float singleSidedAmplitude =
        magnitude * ((bin == 0 || bin == NYQUIST_BIN) ? 1.0f : 2.0f) /
        windowSum;
    bandPower[band] += singleSidedAmplitude * singleSidedAmplitude;
    ++bandBinCount[band];
  }

  for (size_t band = 0; band < AUDIO_SPECTRUM_BAND_COUNT; ++band) {
    float dbfs = AUDIO_SPECTRUM_FLOOR_DBFS;
    if (bandBinCount[band] > 0 && bandPower[band] > 0.0f) {
      const float rmsAmplitude =
          sqrtf(bandPower[band] / static_cast<float>(bandBinCount[band]));
      dbfs = 20.0f * log10f(rmsAmplitude);
    }
    if (!std::isfinite(dbfs) || dbfs < AUDIO_SPECTRUM_FLOOR_DBFS) {
      dbfs = AUDIO_SPECTRUM_FLOOR_DBFS;
    } else if (dbfs > AUDIO_SPECTRUM_CEILING_DBFS) {
      dbfs = AUDIO_SPECTRUM_CEILING_DBFS;
    }
    bandsDbfs[band] = dbfs;
  }
}

inline bool formatAudioSpectrumPacket(
    char *output, size_t outputSize, uint64_t timeMs, uint32_t sampleRateHz,
    const float bandsDbfs[AUDIO_SPECTRUM_BAND_COUNT]) {
  if (output == nullptr || outputSize == 0) return false;

  int written = snprintf(
      output, outputSize,
      "{\"type\":\"audio_spectrum\",\"time_ms\":%llu,\"sample_rate_hz\":%lu,"
      "\"fft_size\":%u,\"min_hz\":0,\"max_hz\":%lu,\"scale\":\"linear\","
      "\"audio_fft\":[",
      static_cast<unsigned long long>(timeMs),
      static_cast<unsigned long>(sampleRateHz),
      static_cast<unsigned>(AUDIO_SPECTRUM_FFT_SIZE),
      static_cast<unsigned long>(sampleRateHz / 2));
  if (written < 0 || static_cast<size_t>(written) >= outputSize) return false;
  size_t used = static_cast<size_t>(written);

  for (size_t band = 0; band < AUDIO_SPECTRUM_BAND_COUNT; ++band) {
    float value = bandsDbfs[band];
    if (!std::isfinite(value) || value < AUDIO_SPECTRUM_FLOOR_DBFS) {
      value = AUDIO_SPECTRUM_FLOOR_DBFS;
    } else if (value > AUDIO_SPECTRUM_CEILING_DBFS) {
      value = AUDIO_SPECTRUM_CEILING_DBFS;
    }
    written = snprintf(output + used, outputSize - used, "%s%.1f",
                       band == 0 ? "" : ",", static_cast<double>(value));
    if (written < 0 || static_cast<size_t>(written) >= outputSize - used) return false;
    used += static_cast<size_t>(written);
  }

  written = snprintf(output + used, outputSize - used, "]}");
  return written == 2 && used + static_cast<size_t>(written) < outputSize;
}

}  // namespace pal
