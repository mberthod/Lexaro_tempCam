// telemetry.h — Native WebSocket server/client and JSON publishing
#pragma once

#include <Arduino.h>
#include <functional>
#include "../drivers/ir_manager.h"
#include "../drivers/radar_manager.h"
#include "../core/fusion_engine.h"

enum class TelemetryNetMode {
  Server,
  Client
};

void telemetry_begin_server(const char* path, uint16_t port);
void telemetry_stop_server();
bool telemetry_is_server_running();
void telemetry_set_mode(TelemetryNetMode mode);
bool telemetry_begin_client(const char* url);
void telemetry_loop();

// Publishing helpers (JSON)
void telemetry_publish_ir_frame(const IRFrame& frame, const IRSummary& summary);
void telemetry_publish_radar(const RadarSample& sample);
void telemetry_publish_fusion(const TracksView& view);
void telemetry_publish_event(const char* kind, uint32_t trackId, float severity);

// Logging (single line, spaces percent-encoded as %20)
void telemetry_log(const char* msg);
void telemetry_logf(const char* fmt, ...);


