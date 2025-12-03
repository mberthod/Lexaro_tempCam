#pragma once

#include <Arduino.h>

struct OtaConfig {
  String url;     // URL du binaire OTA (ex: https://github.com/<user>/<repo>/releases/latest/download/firmware.bin)
  bool autoCheck; // true => tentative au boot si Wi‑Fi OK
};

bool ota_init();                       // charge la config (Preferences)
void ota_set_url(const String& url);   // persiste l’URL
String ota_get_url();
void ota_set_auto(bool en);            // persiste le flag auto
bool ota_get_auto();
bool ota_run_now(String* errMsg);      // lance l’OTA immédiat (bloquant, redémarre si succès). Retourne true si mise à jour appliquée.
String ota_status();                   // renvoie un court état textuel


