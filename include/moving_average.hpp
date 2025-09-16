#include <Arduino.h>

#pragma once

template <uint8_t N>
class MovingAverage {
 public:
  MovingAverage() : sum(0), idx(0) {
    static_assert((N & (N - 1)) == 0, "N must be a power of two");
    for (auto &v : buf) v = 0;
  }

  float update(float sample) {
    sum -= buf[idx];
    buf[idx] = sample;
    sum += sample;
    idx = (idx + 1) & (N - 1);
    return sum;
  }

 private:
  float buf[N];
  float sum;
  uint8_t idx;
};