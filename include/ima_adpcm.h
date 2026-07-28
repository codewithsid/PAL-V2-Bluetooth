#ifndef IMA_ADPCM_H
#define IMA_ADPCM_H

#include <cstdint>
#include <cstddef>
#include <algorithm>

namespace ImaAdpcmConstants {
static const int STEP_TABLE[89] = {
  7, 8, 9, 10, 11, 12, 13, 14, 16, 17,
  19, 21, 23, 25, 28, 31, 34, 37, 41, 45,
  50, 55, 60, 66, 73, 80, 88, 97, 107, 118,
  130, 143, 157, 173, 190, 209, 230, 253, 279, 307,
  337, 371, 408, 449, 494, 544, 598, 658, 724, 796,
  876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066,
  2272, 2499, 2749, 3024, 3326, 3659, 4025, 4428, 4870, 5357,
  5893, 6482, 7130, 7843, 8627, 9490, 10439, 11483, 12631, 13895,
  15285, 16814, 18495, 20345, 22380, 24618, 27079, 29787, 32767
};

static const int INDEX_TABLE[16] = {
  -1, -1, -1, -1, 2, 4, 6, 8,
  -1, -1, -1, -1, 2, 4, 6, 8
};
}

class ImaAdpcmEncoder {
public:
  ImaAdpcmEncoder() : predictedSample(0), stepIndex(0) {}

  void reset() {
    predictedSample = 0;
    stepIndex = 0;
  }

  uint8_t encodeSample(int16_t sample) {
    int step = ImaAdpcmConstants::STEP_TABLE[stepIndex];
    int diff = sample - predictedSample;
    uint8_t nibble = 0;

    if (diff < 0) {
      nibble = 8;
      diff = -diff;
    }

    int delta = step >> 3;
    if (diff >= step) {
      nibble |= 4;
      diff -= step;
      delta += step;
    }
    if (diff >= (step >> 1)) {
      nibble |= 2;
      diff -= (step >> 1);
      delta += (step >> 1);
    }
    if (diff >= (step >> 2)) {
      nibble |= 1;
      delta += (step >> 2);
    }

    if (nibble & 8) {
      predictedSample -= delta;
    } else {
      predictedSample += delta;
    }

    if (predictedSample > 32767) predictedSample = 32767;
    if (predictedSample < -32768) predictedSample = -32768;

    stepIndex += ImaAdpcmConstants::INDEX_TABLE[nibble];
    if (stepIndex < 0) stepIndex = 0;
    if (stepIndex > 88) stepIndex = 88;

    return nibble;
  }

  size_t encodeBlock(const int16_t *pcmInput, size_t numSamples, uint8_t *adpcmOutput) {
    size_t outBytes = 0;
    for (size_t i = 0; i < numSamples; i += 2) {
      uint8_t n0 = encodeSample(pcmInput[i]);
      uint8_t n1 = (i + 1 < numSamples) ? encodeSample(pcmInput[i + 1]) : 0;
      adpcmOutput[outBytes++] = n0 | (n1 << 4);
    }
    return outBytes;
  }

  int16_t getPredictedSample() const { return static_cast<int16_t>(predictedSample); }
  int getStepIndex() const { return stepIndex; }

private:
  int32_t predictedSample;
  int stepIndex;
};

class ImaAdpcmDecoder {
public:
  ImaAdpcmDecoder() : predictedSample(0), stepIndex(0) {}

  void reset() {
    predictedSample = 0;
    stepIndex = 0;
  }

  int16_t decodeNibble(uint8_t nibble) {
    int step = ImaAdpcmConstants::STEP_TABLE[stepIndex];
    int delta = step >> 3;
    if (nibble & 4) delta += step;
    if (nibble & 2) delta += (step >> 1);
    if (nibble & 1) delta += (step >> 2);

    if (nibble & 8) {
      predictedSample -= delta;
    } else {
      predictedSample += delta;
    }

    if (predictedSample > 32767) predictedSample = 32767;
    if (predictedSample < -32768) predictedSample = -32768;

    stepIndex += ImaAdpcmConstants::INDEX_TABLE[nibble & 0x0F];
    if (stepIndex < 0) stepIndex = 0;
    if (stepIndex > 88) stepIndex = 88;

    return static_cast<int16_t>(predictedSample);
  }

  size_t decodeBlock(const uint8_t *adpcmInput, size_t inputBytes, int16_t *pcmOutput) {
    size_t outSamples = 0;
    for (size_t i = 0; i < inputBytes; ++i) {
      uint8_t byte = adpcmInput[i];
      pcmOutput[outSamples++] = decodeNibble(byte & 0x0F);
      pcmOutput[outSamples++] = decodeNibble((byte >> 4) & 0x0F);
    }
    return outSamples;
  }

  int16_t getPredictedSample() const { return static_cast<int16_t>(predictedSample); }
  int getStepIndex() const { return stepIndex; }

private:
  int32_t predictedSample;
  int stepIndex;
};

#endif // IMA_ADPCM_H
