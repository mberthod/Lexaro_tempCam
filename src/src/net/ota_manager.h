#pragma once

#include <Arduino.h>

/**
 * @file ota_manager.h
 * @brief Gestion des mises à jour OTA HTTP(S) depuis GitHub Releases.
 *
 * - Suivi des redirections HTTP 30x.
 * - Logs monoligne percent-encodés pour compat WS [[memory:8240683]].
 * - Redémarrage explicite après succès pour garantir l’émission des logs finaux.
 */

/**
 * @brief Configuration OTA persistée.
 */
struct OtaConfig {
  String url;       ///< URL binaire OTA (ex: https://github.com/<user>/<repo>/releases/latest/download/firmware.bin)
  bool  autoCharge; ///< Réservé (non utilisé, conservé pour compatibilité binaire)
  bool  autoCheck;  ///< true => tentative au boot si Wi‑Fi OK
};

/**
 * @brief Initialise le gestionnaire OTA et charge la configuration.
 * @return true si OK
 */
bool   ota_init();

/**
 * @brief Définit l’URL OTA (persistée).
 * @param url URL complète (HTTP/HTTPS)
 */
void   ota_set_url(const String& url);

/**
 * @brief Retourne l’URL OTA courante.
 */
String ota_get_url();

/**
 * @brief Active/désactive l’OTA automatique au boot.
 * @param en true pour activer
 */
void   ota_set_auto(bool en);

/**
 * @brief Indique si l’OTA auto est activée.
 */
bool   ota_get_auto();

/**
 * @brief Lance l’OTA de manière synchrone (bloquant).
 * @param errMsg Chaîne optionnelle recevant un message d’erreur
 * @return true si mise à jour appliquée (l’appareil redémarre ensuite)
 */
bool   ota_run_now(String* errMsg);

/**
 * @brief Chaîne de statut lisible (url, auto, running, verbose).
 */
String ota_status();

/**
 * @brief Indique si une mise à jour est en cours.
 */
bool   ota_is_running();

/**
 * @brief Active le mode verbeux OTA (traces détaillées).
 */
void   ota_set_verbose(bool en);

/**
 * @brief Retourne l’état du mode verbeux OTA.
 */
bool   ota_get_verbose();


