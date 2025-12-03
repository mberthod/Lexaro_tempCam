// wifi_manager.h — minimal Wi‑Fi manager using saved credentials and WS commands
#pragma once

#include <Arduino.h>

bool wifi_manager_begin();               // load saved creds and try connect (keeps AP active)
bool wifi_manager_connect(const char* ssid, const char* pass, bool save); // connect and optionally save
const char* wifi_manager_status();       // short status string
void wifi_manager_loop();                // optional maintenance

// WiFiManager captive portal (non-bloquant)
bool wifi_manager_start_portal(const char* apName, const char* apPass, uint32_t timeoutMs);
void wifi_manager_stop_portal();
bool wifi_manager_portal_running();


