// calibration.h — IR temperature calibration utilities
#pragma once

#include <Arduino.h>
#include "../drivers/ir_manager.h"

// Apply a simple ambient-based offset/gain to IR pixels and update center samples
static inline void calibration_apply_ambient(IRFrame& frame, float ambientC) {
  const float amb = isnan(ambientC) ? 25.0f : ambientC;
  const int N = frame.width * frame.height;
  for (int i = 0; i < N; ++i) {
    // Example: pull frame baseline slightly towards ambient (simple drift compensation)
    const float t = frame.pixelsC[i];
    const float tAdj = t + 0.02f * (amb - t);
    frame.pixelsC[i] = tAdj;
  }
  // Recompute center samples
  const int cx = frame.width / 2;
  const int cy = frame.height / 2;
  frame.centerValuesC[0] = frame.pixelsC[cy * frame.width + cx];
  frame.centerValuesC[1] = frame.pixelsC[cy * frame.width + max(0, cx - 1)];
  frame.centerValuesC[2] = frame.pixelsC[cy * frame.width + min((int)frame.width - 1, cx + 1)];
  frame.centerValuesC[3] = frame.pixelsC[max(0, cy - 1) * frame.width + cx];
  frame.centerValuesC[4] = frame.pixelsC[min((int)frame.height - 1, cy + 1) * frame.width + cx];
}

// Simple distance/angle compensation (skeleton):
// attenuate with 1/r^2 and cosine of incidence angle; clamp factor within reasonable bounds.
static inline float compensate_distance_angle(float value, float distanceM, float angleRad) {
  const float r = max(0.2f, distanceM); // avoid zero
  const float cosA = cosf(angleRad);
  const float factor = (cosA <= 0.0f) ? 0.1f : (cosA / (r * r));
  const float k = constrain(factor, 0.1f, 5.0f);
  return value * k;
}


