// wifi_manager.cpp — minimal Wi‑Fi manager (STA connect with saved credentials)
#include "wifi_manager.h"
#include <Preferences.h>
#include <WiFiManager.h>
#include <WiFi.h>

static Preferences s_prefs;
static String s_ssid;
static String s_pass;
static String s_status = "idle";
static WiFiManager* s_wm = nullptr;
static bool s_portalRunning = false;
static uint32_t s_portalDeadlineMs = 0;

static void update_status() {
  if (WiFi.status() == WL_CONNECTED) {
    s_status = "connected";
  } else {
    s_status = "disconnected";
  }
}

bool wifi_manager_begin() {
  s_prefs.begin("lexaro", false);
  s_ssid = s_prefs.getString("sta_ssid", "");
  s_pass = s_prefs.getString("sta_pass", "");
  if (s_ssid.length() > 0) {
    WiFi.mode(WIFI_AP_STA);
    WiFi.begin(s_ssid.c_str(), s_pass.c_str());
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 8000) {
      delay(200);
    }
  }
  update_status();
  return WiFi.status() == WL_CONNECTED;
}

bool wifi_manager_connect(const char* ssid, const char* pass, bool save) {
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(ssid, pass);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
    delay(200);
  }
  bool ok = (WiFi.status() == WL_CONNECTED);
  if (ok && save) {
    s_prefs.putString("sta_ssid", ssid);
    s_prefs.putString("sta_pass", pass);
    s_ssid = ssid; s_pass = pass;
  }
  update_status();
  return ok;
}

const char* wifi_manager_status() {
  update_status();
  return s_status.c_str();
}

void wifi_manager_loop() {
  if (s_portalRunning && s_wm) {
    s_wm->process();
    if (WiFi.status() == WL_CONNECTED) {
      // Save credentials
      s_prefs.putString("sta_ssid", WiFi.SSID());
      // Password not retrievable; keep existing if set via CLI/WS
      s_portalRunning = false;
      s_wm->stopConfigPortal();
    } else if (s_portalDeadlineMs && millis() > s_portalDeadlineMs) {
      s_portalRunning = false;
      s_wm->stopConfigPortal();
    }
  }
}

bool wifi_manager_start_portal(const char* apName, const char* apPass, uint32_t timeoutMs) {
  if (!s_wm) s_wm = new WiFiManager();
  // Basculer en mode AP seul et IP statique 192.168.4.1 pour fiabiliser l'accès
  WiFi.disconnect(true, true);
  delay(100);
  WiFi.mode(WIFI_AP);
  IPAddress apIP(192, 168, 4, 1);
  IPAddress gw(192, 168, 4, 1);
  IPAddress sn(255, 255, 255, 0);
  WiFi.softAPConfig(apIP, gw, sn);

  s_wm->setCaptivePortalEnable(true);
  s_wm->setEnableConfigPortal(true);
  s_wm->setConfigPortalBlocking(false);
  if (timeoutMs > 0) s_wm->setConfigPortalTimeout(timeoutMs / 1000);
  String ap = apName && *apName ? apName : String("LexaroCfg-") + String((uint32_t)ESP.getEfuseMac(), HEX);
  const char* pass = (apPass && *apPass) ? apPass : "lexaro123"; // WPA2 pour meilleurs déclenchements Android
  s_wm->startConfigPortal(ap.c_str(), pass);
  // With non-blocking mode, startConfigPortal returns 'connected' status; portal may still be active.
  s_portalRunning = s_wm->getConfigPortalActive();
  s_portalDeadlineMs = timeoutMs ? (millis() + timeoutMs) : 0;
  return true;
}

void wifi_manager_stop_portal() {
  if (s_wm) s_wm->stopConfigPortal();
  s_portalRunning = false;
  s_portalDeadlineMs = 0;
  // Revenir en mode AP+STA (l'AP SoftAP de l'app reste actif)
  WiFi.mode(WIFI_AP_STA);
}

bool wifi_manager_portal_running() { return s_portalRunning; }


