// Filters and estimators (IIR bandpass for respiration, simple estimators)
#pragma once

#include <Arduino.h>
#include <math.h>
#include <stdint.h>

class Biquad {
public:
  Biquad() { reset(); }
  void reset() { x1 = x2 = y1 = y2 = 0.0f; b0 = b1 = b2 = a1 = a2 = 0.0f; }
  float process(float x) {
    const float y = b0*x + b1*x1 + b2*x2 - a1*y1 - a2*y2;
    x2 = x1; x1 = x; y2 = y1; y1 = y;
    return y;
  }
  void setCoefficients(float b0_in, float b1_in, float b2_in, float a1_in, float a2_in) {
    b0 = b0_in; b1 = b1_in; b2 = b2_in; a1 = a1_in; a2 = a2_in;
  }
private:
  float x1, x2, y1, y2;
  float b0, b1, b2, a1, a2;
};

// Simple respiration band-pass (0.1–1.0 Hz) at ~30 Hz sampling
class RespirationIIRFilter {
public:
  void configureDefault() {
    // Two cascaded biquads approximating a 2nd–4th order band-pass
    // Coeffs generated offline for fs=30Hz (Butterworth-like), normalized (a0=1)
    // Stage 1
    s1.setCoefficients(0.965080f, -1.930160f, 0.965080f, -1.929404f, 0.931078f);
    // Stage 2
    s2.setCoefficients(0.982386f, -1.964773f, 0.982386f, -1.964562f, 0.965017f);
  }
  float process(float i, float q) {
    // Magnitude as proxy of motion in respiration band
    const float mag = sqrtf(i*i + q*q);
    const float y1 = s1.process(mag);
    const float y2 = s2.process(y1);
    return y2;
  }
private:
  Biquad s1, s2;
};

// Respiration estimator using zero-crossing period estimation with smoothing
class RespirationEstimator {
public:
  RespirationEstimator() { reset(); }
  void reset() {
    lastSign = 0;
    lastZeroUs = 0;
    bpm = 0.0f;
  }
  float update(uint64_t tsUs, float sample) {
    const int sign = (sample >= 0.0f) ? 1 : -1;
    if (lastSign == -1 && sign == 1) {
      if (lastZeroUs != 0) {
        const float periodSec = (tsUs - lastZeroUs) * 1e-6f;
        const float instBpm = 60.0f / max(periodSec, 0.1f);
        // Smooth
        bpm = 0.8f * bpm + 0.2f * instBpm;
      }
      lastZeroUs = tsUs;
    }
    lastSign = sign;
    return bpm;
  }
private:
  int lastSign;
  uint64_t lastZeroUs;
  float bpm;
};


