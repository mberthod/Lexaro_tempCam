// telemetry.h — Native WebSocket server/client and JSON publishing
#pragma once

#include <Arduino.h>
#include <functional>
#include "../drivers/ir_manager.h"
#include "../drivers/radar_manager.h"
#include "../core/fusion_engine.h"

/**
 * @file telemetry.h
 * @brief Serveur HTTPD + WebSocket natif et client WebSocket sortant.
 *
 * - Publie les frames IR/radar/fusion au format JSON.
 * - Redirige les logs sur une seule ligne en percent-encoding des espaces [[memory:8240683]].
 * - Peut fonctionner en mode serveur (ws://device/ws) ou client (vers un hôte).
 */

/**
 * @brief Mode réseau de la télémétrie.
 */
enum class TelemetryNetMode {
  Server,
  Client
};

/**
 * @brief Démarre le serveur HTTP/WS local.
 * @param path Chemin WS (ex: "/ws")
 * @param port Port HTTP (ex: 80)
 */
void telemetry_begin_server(const char* path, uint16_t port);

/**
 * @brief Stoppe le serveur HTTP/WS local.
 */
void telemetry_stop_server();

/**
 * @brief Indique si le serveur est actif.
 */
bool telemetry_is_server_running();

/**
 * @brief Définit le mode réseau (serveur ou client).
 */
void telemetry_set_mode(TelemetryNetMode mode);

/**
 * @brief Démarre le client WebSocket sortant.
 * @param url URL ws:// ou wss://
 * @return true si connecté
 */
bool telemetry_begin_client(const char* url);

/**
 * @brief Boucle d’entretien (à appeler régulièrement dans une tâche).
 */
void telemetry_loop();

// Publishing helpers (JSON)
/**
 * @brief Publie une frame IR et son résumé (WS + MQTT).
 */
void telemetry_publish_ir_frame(const IRFrame& frame, const IRSummary& summary);
/**
 * @brief Publie un échantillon radar (WS + MQTT).
 */
void telemetry_publish_radar(const RadarSample& sample);
/**
 * @brief Publie l’état de fusion (pistes) (WS + MQTT).
 */
void telemetry_publish_fusion(const TracksView& view);
/**
 * @brief Publie un évènement (WS + MQTT).
 */
void telemetry_publish_event(const char* kind, uint32_t trackId, float severity);

// Logging (single line, spaces percent-encoded as %20)
/**
 * @brief Log textuel monoligne. Espaces encodés en %20.
 */
void telemetry_log(const char* msg);
/**
 * @brief Variante printf pour le log.
 */
void telemetry_logf(const char* fmt, ...);


