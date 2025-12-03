// IR Manager for MLX90640 (skeleton, with synthetic fallback)
#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "../config/Config.h"
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"

struct IRSummary {
  bool valid;
  float centroidX;
  float centroidY;
  float tMaxC;
  float area;
};

struct IRFrame {
  uint64_t timestampUs;
  uint16_t width;
  uint16_t height;
  float pixelsC[IR_HEIGHT * IR_WIDTH]; // row-major
  float centerValuesC[5]; // cross sample around center
};

struct IRSample {
  uint64_t timestampUs;
  float ambientC;
  uint16_t width;
  uint16_t height;
  float centerValuesC[5];
  IRSummary summary;
};

class IRManager {
public:
  IRManager() : _wire(nullptr), _ok(false) {}
  bool begin(TwoWire& wire);
  bool captureFrame(IRFrame& out);
  void syntheticFrame(IRFrame& out, float ambientC);
  bool extractSummary(const IRFrame& in, IRSummary& outSummary);
  bool ok() const { return _ok; }
private:
  TwoWire* _wire;
  bool _ok;
  paramsMLX90640 _params;
  // Buffers alloués en statique (évite l'overflow de pile dans TaskIR)
  uint16_t _frameRaw[834];
  float _to[IR_HEIGHT * IR_WIDTH];
  void computeCenterSamples(const IRFrame& in, float* center5) const;
};


