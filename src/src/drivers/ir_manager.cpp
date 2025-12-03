// IR Manager implementation (MLX90640 skeleton)
#include "ir_manager.h"
#include <math.h>

bool IRManager::begin(TwoWire& wire) {
  _wire = &wire;
  // Quick ping
  _wire->beginTransmission(MLX90640_I2C_ADDR);
  int ack = _wire->endTransmission();
  Serial.printf("[MLX] ping endTx=%d (0=OK)\n", ack);
  if (ack != 0) { _ok = false; return false; }
  // Configure I2C frequency for MLX
  MLX90640_I2CFreqSet(400000);
  // Load EEPROM and extract parameters
  uint16_t eeData[832] = {0};
  int st = MLX90640_DumpEE(MLX90640_I2C_ADDR, eeData);
  if (st != 0) { Serial.printf("[MLX] DumpEE err=%d\n", st); _ok = false; return false; }
  st = MLX90640_ExtractParameters(eeData, &_params);
  if (st != 0) { Serial.printf("[MLX] ExtractParameters err=%d\n", st); _ok = false; return false; }
  // Chess mode + 8Hz
  MLX90640_SetChessMode(MLX90640_I2C_ADDR);
  MLX90640_SetRefreshRate(MLX90640_I2C_ADDR, 0x04);
  _ok = true;
  return true;
}

bool IRManager::captureFrame(IRFrame& out) {
  if (!_ok) return false;
  memset(_frameRaw, 0, sizeof(_frameRaw));
  int st = MLX90640_GetFrameData(MLX90640_I2C_ADDR, _frameRaw);
  if (st < 0) { Serial.printf("[MLX] GetFrameData err=%d\n", st); return false; }
  float Ta = MLX90640_GetTa(_frameRaw, &_params);
  float emissivity = 0.95f;
  MLX90640_CalculateTo(_frameRaw, &_params, emissivity, Ta, _to);
  // Copy to out
  out.width = IR_WIDTH;
  out.height = IR_HEIGHT;
  for (int i = 0; i < IR_WIDTH * IR_HEIGHT; ++i) out.pixelsC[i] = _to[i];
  computeCenterSamples(out, out.centerValuesC);
  return true;
}

void IRManager::syntheticFrame(IRFrame& out, float ambientC) {
  const float base = isnan(ambientC) ? 25.0f : ambientC;
  // Create a synthetic warm blob moving slowly
  const uint64_t tms = (out.timestampUs / 1000ULL);
  const float cx = (IR_WIDTH / 2.0f) + 6.0f * sinf(tms * 0.001f);
  const float cy = (IR_HEIGHT / 2.0f) + 3.0f * cosf(tms * 0.0012f);
  const float amp = 2.0f + 1.0f * sinf(tms * 0.0007f);
  for (uint16_t y = 0; y < IR_HEIGHT; ++y) {
    for (uint16_t x = 0; x < IR_WIDTH; ++x) {
      const float dx = (float)x - cx;
      const float dy = (float)y - cy;
      const float d2 = dx*dx + dy*dy;
      float t = base + amp * expf(-d2 / 25.0f);
      out.pixelsC[y * IR_WIDTH + x] = t;
    }
  }
  computeCenterSamples(out, out.centerValuesC);
}

bool IRManager::extractSummary(const IRFrame& in, IRSummary& outSummary) {
  // Simple adaptive threshold above median
  float minT = 1e9f, maxT = -1e9f, sum = 0.0f;
  const int N = IR_WIDTH * IR_HEIGHT;
  for (int i = 0; i < N; ++i) {
    float v = in.pixelsC[i];
    minT = min(minT, v);
    maxT = max(maxT, v);
    sum += v;
  }
  const float avg = sum / (float)N;
  const float thr = avg + 0.8f; // 0.8°C above average

  float accX = 0.0f, accY = 0.0f, accW = 0.0f;
  float tMax = -1e9f;
  for (uint16_t y = 0; y < IR_HEIGHT; ++y) {
    for (uint16_t x = 0; x < IR_WIDTH; ++x) {
      const float t = in.pixelsC[y * IR_WIDTH + x];
      if (t > thr) {
        accX += x;
        accY += y;
        accW += 1.0f;
        if (t > tMax) tMax = t;
      }
    }
  }
  if (accW < 3.0f) {
    outSummary.valid = false;
    outSummary.centroidX = 0;
    outSummary.centroidY = 0;
    outSummary.tMaxC = maxT;
    outSummary.area = 0;
    return false;
  }
  outSummary.valid = true;
  outSummary.centroidX = accX / accW;
  outSummary.centroidY = accY / accW;
  outSummary.tMaxC = tMax;
  outSummary.area = accW;
  return true;
}

void IRManager::computeCenterSamples(const IRFrame& in, float* center5) const {
  const int cx = IR_WIDTH / 2;
  const int cy = IR_HEIGHT / 2;
  center5[0] = in.pixelsC[cy * IR_WIDTH + cx];
  center5[1] = in.pixelsC[cy * IR_WIDTH + max(0, cx - 1)];
  center5[2] = in.pixelsC[cy * IR_WIDTH + min((int)IR_WIDTH - 1, cx + 1)];
  center5[3] = in.pixelsC[max(0, cy - 1) * IR_WIDTH + cx];
  center5[4] = in.pixelsC[min((int)IR_HEIGHT - 1, cy + 1) * IR_WIDTH + cx];
}


