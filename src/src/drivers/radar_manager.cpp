// Radar Manager implementation (BGT60TR13C skeleton)
#include "radar_manager.h"
#include <math.h>
#include <stdlib.h>

bool RadarManager::begin(SPIClass& spi) {
  _spi = &spi;
  // In a full implementation: reset, configure FMCW profile, ramps, sampling, DMA
  _ok = false; // mark not ready; synthetic used for now
  return true;
}

bool RadarManager::sampleRaw(RadarRaw& out) {
  if (!_ok) return false;
  // Real acquisition would retrieve I/Q samples; not implemented here
  return false;
}

void RadarManager::syntheticRaw(RadarRaw& out, uint64_t tsUs) {
  // Simulate respiration ~ 18 bpm (0.3 Hz) with small noise and presence toggling
  const float t = tsUs * 1e-6f;
  const float freq = 0.30f; // Hz
  const float phase = 2.0f * (float)M_PI * freq * t;
  const float signal = 0.8f * sinf(phase) + 0.05f * sinf(2 * phase);
  out.iComponent = signal + 0.02f * (float)rand() / RAND_MAX;
  out.qComponent = 0.9f * cosf(phase) + 0.02f * (float)rand() / RAND_MAX;
  out.motionRms = fabsf(signal) * 0.2f + 0.02f;
  // Presence: 8s on, 4s off
  const int period = ((int)t) % 12;
  out.presence = (period < 8);
}


