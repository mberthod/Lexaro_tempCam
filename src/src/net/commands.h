// commands.h — percent-encoded monoline commands over WebSocket
#pragma once

#include <Arduino.h>
#include <functional>
#include "telemetry.h"

/**
 * @file commands.h
 * @brief Parser de commandes monoligne pour WebSocket (percent-encoded) et USB.
 *
 * Exemple WS: "/mqtt%20set?host=192.168.0.10&port=1883" (espaces -> %20).
 */

using SetU16Func = std::function<void(uint16_t)>;
using SetModeFunc = std::function<void(TelemetryNetMode)>;

/**
 * @brief Lie des callbacks pour la configuration runtime (FPS, mode réseau).
 */
void commands_bind_setters(SetU16Func setIrFps, SetU16Func setRadarFps, SetModeFunc setMode);

/**
 * @brief Traite une ligne de commande (WS/USB). Les séquences %xx sont décodées.
 */
void commands_handle_line(const char* line);


