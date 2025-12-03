// commands.h — percent-encoded monoline commands over WebSocket
#pragma once

#include <Arduino.h>
#include <functional>
#include "telemetry.h"

using SetU16Func = std::function<void(uint16_t)>;
using SetModeFunc = std::function<void(TelemetryNetMode)>;

void commands_bind_setters(SetU16Func setIrFps, SetU16Func setRadarFps, SetModeFunc setMode);
void commands_handle_line(const char* line);


