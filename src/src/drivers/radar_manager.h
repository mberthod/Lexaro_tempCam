// Radar Manager for BGT60TR13C (skeleton with synthetic fallback)
#pragma once

#include <Arduino.h>
#include <SPI.h>
#include "../config/Config.h"

struct RadarRaw {
  float iComponent;
  float qComponent;
  bool presence;
  float motionRms;
};

struct RadarSample {
  uint64_t timestampUs;
  bool presence;
  float motionRms;
  float filtered;
  float respBpm;
};

class RadarManager {
public:
  RadarManager() : _spi(nullptr), _ok(false) {}
  bool begin(SPIClass& spi);
  bool sampleRaw(RadarRaw& out);
  void syntheticRaw(RadarRaw& out, uint64_t tsUs);
  bool ok() const { return _ok; }
private:
  SPIClass* _spi;
  bool _ok;
};


