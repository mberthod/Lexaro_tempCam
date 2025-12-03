// fusion_engine.h — Simple multi-sensor fusion (EKF skeleton)
#pragma once

#include <Arduino.h>
#include <vector>
#include "../drivers/ir_manager.h"
#include "../drivers/radar_manager.h"

struct Track {
  uint32_t id;
  bool active;
  float x;
  float y;
  float vx;
  float vy;
  float respBpm;
  float tBody;
  float confidence;
  uint64_t lastUpdateUs;
};

struct TracksView {
  const Track* tracks;
  size_t count;
};

class FusionEngine {
public:
  void begin() {
    _tracks.reserve(2);
    _nextId = 1;
  }
  void update(const IRSample* ir, const RadarSample* radar) {
    // Birth: one track if none and IR has a valid blob
    if (_tracks.empty()) {
      if (ir && ir->summary.valid) {
        Track t{};
        t.id = _nextId++;
        t.active = true;
        t.x = ir->summary.centroidX;
        t.y = ir->summary.centroidY;
        t.vx = 0; t.vy = 0;
        t.respBpm = radar ? radar->respBpm : 0;
        t.tBody = ir->summary.tMaxC;
        t.confidence = 0.7f;
        t.lastUpdateUs = ir->timestampUs;
        _tracks.push_back(t);
      }
    } else {
      // Single-track update (skeleton)
      Track& t = _tracks[0];
      const uint64_t nowUs = (ir ? ir->timestampUs : (radar ? radar->timestampUs : t.lastUpdateUs));
      const float dt = (nowUs - t.lastUpdateUs) * 1e-6f;
      // Predict
      t.x += t.vx * dt;
      t.y += t.vy * dt;
      // IR measurement update
      if (ir && ir->summary.valid) {
        const float k = 0.5f;
        t.vx = (ir->summary.centroidX - t.x) / max(1e-3f, dt);
        t.vy = (ir->summary.centroidY - t.y) / max(1e-3f, dt);
        t.x = t.x + k * (ir->summary.centroidX - t.x);
        t.y = t.y + k * (ir->summary.centroidY - t.y);
        t.tBody = 0.8f * t.tBody + 0.2f * ir->summary.tMaxC;
        t.confidence = min(1.0f, t.confidence + 0.05f);
      }
      // Radar measurement update
      if (radar) {
        t.respBpm = 0.8f * t.respBpm + 0.2f * radar->respBpm;
      }
      t.lastUpdateUs = nowUs;
      // Simple deactivation rule
      if (!ir && (!radar || !radar->presence)) {
        t.confidence = max(0.0f, t.confidence - 0.02f);
        if (t.confidence < 0.05f) {
          t.active = false;
        }
      } else {
        t.active = true;
      }
    }
    // Event detection placeholder (e.g., apnea)
    if (!_tracks.empty()) {
      Track& t = _tracks[0];
      if (t.active && t.respBpm < 3.0f) {
        // In a full implementation, maintain a timer; here we only flag low bpm instantaneously
        _lastEventKind = "apnea";
      } else {
        _lastEventKind = nullptr;
      }
    }
  }
  TracksView getTracksView() const {
    return TracksView{ _tracks.data(), _tracks.size() };
  }
  const char* getLastEventKind() const { return _lastEventKind; }
private:
  std::vector<Track> _tracks;
  uint32_t _nextId = 1;
  const char* _lastEventKind = nullptr;
};


