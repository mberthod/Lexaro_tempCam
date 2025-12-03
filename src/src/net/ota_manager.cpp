#include "ota_manager.h"
#include <Preferences.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include "telemetry.h"

static Preferences s_prefs;
static OtaConfig s_cfg;
static volatile bool s_running = false;
static volatile bool s_verbose = true; // logs détaillés activés

// Encodage simple des espaces -> %20 pour respecter le format de logs
static String ota_percent_encode(const String& in) {
  String out; out.reserve(in.length() + 16);
  for (size_t i = 0; i < in.length(); ++i) {
    char c = in[i];
    if (c == ' ') out += "%20";
    else out += c;
  }
  return out;
}

static void ota_vlog(const String& msg) {
  if (!s_verbose) return;
  String enc = ota_percent_encode(msg);
  telemetry_log(enc.c_str());
  // Miroir sur USB CDC pour visibilité même sans client WS
  Serial.println(enc);
}

// Effectue un GET sans suivi de redirection pour journaliser les 30x éventuels.
static String ota_resolve_redirect_once(const String& url, int* httpCodeOut) {
  WiFiClientSecure c;
  c.setInsecure();
  c.setTimeout(8000);
  HTTPClient http;
  http.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
  ota_vlog(String("OTA preflight begin url=") + url);
  if (!http.begin(c, url)) {
    ota_vlog("OTA preflight begin fail");
    if (httpCodeOut) *httpCodeOut = -1;
    return url;
  }
  int code = http.GET();
  if (httpCodeOut) *httpCodeOut = code;
  ota_vlog(String("OTA preflight code=") + code);
  String finalUrl = url;
  if (code >= 300 && code < 400) {
    String loc = http.header("Location");
    ota_vlog(String("OTA redirect location=") + loc);
    if (loc.length()) finalUrl = loc;
  }
  http.end();
  return finalUrl;
}

bool ota_init() {
  s_prefs.begin("lexaro", false);
  s_cfg.url = s_prefs.getString("ota_url", "");
  s_cfg.autoCheck = s_prefs.getBool("ota_auto", false);
  if (s_cfg.url.length() == 0) {
    // URL par défaut: asset public "firmware.bin" de la release la plus récente
    s_cfg.url = "https://github.com/mberthod/Lexaro_tempCam/releases/latest/download/firmware.bin";
  }
  return true;
}

void ota_set_url(const String& url) {
  s_cfg.url = url;
  s_prefs.putString("ota_url", url);
  ota_vlog(String("OTA cfg url set=") + url);
}

String ota_get_url() { return s_cfg.url; }

void ota_set_auto(bool en) {
  s_cfg.autoCheck = en;
  s_prefs.putBool("ota_auto", en);
  ota_vlog(String("OTA cfg auto=") + (en ? "on" : "off"));
}

bool ota_get_auto() { return s_prefs.getBool("ota_auto", false); }

String ota_status() {
  String s = "OTA url=" + (s_cfg.url.length() ? s_cfg.url : String("<none>"));
  s += " auto=";
  s += (s_cfg.autoCheck ? "on" : "off");
  s += " running=";
  s += (s_running ? "yes" : "no");
  s += " verbose=";
  s += (s_verbose ? "on" : "off");
  return s;
}

bool ota_is_running() { return s_running; }

void ota_set_verbose(bool en) {
  s_verbose = en;
  ota_vlog(String("OTA verbose=") + (en ? "on" : "off"));
}

bool ota_get_verbose() { return s_verbose; }

bool ota_run_now(String* errMsg) {
  ota_vlog("OTA run_now begin");
  if (s_cfg.url.length() == 0) {
    if (errMsg) *errMsg = "empty url";
    telemetry_log("ERR%20ota%20empty%20url");
    return false;
  }
  ota_vlog("OTA url check ok");
  if (WiFi.status() != WL_CONNECTED) {
    if (errMsg) *errMsg = "wifi disconnected";
    telemetry_log("ERR%20ota%20wifi");
    return false;
  }

  ota_vlog("OTA start");
  ota_vlog(String("OTA url=") + s_cfg.url);

  // Étape de préflight pour tracer les redirections éventuelles (302)
  int preCode = 0;
  String finalUrl = ota_resolve_redirect_once(s_cfg.url, &preCode);
  ota_vlog(String("OTA final_url=") + finalUrl);

  s_running = true;

  WiFiClientSecure client;
  client.setInsecure();
  client.setTimeout(15000);

  // Laisser le contrôle du reboot pour logguer clairement la fin
  httpUpdate.rebootOnUpdate(false);
  httpUpdate.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

  ota_vlog("OTA update begin");
  t_httpUpdate_return ret = httpUpdate.update(client, finalUrl);

  s_running = false;

  switch (ret) {
    case HTTP_UPDATE_OK:
      ota_vlog("OTA update ok");
      telemetry_log("OTA%20ok");
      // Petite pause pour vider les buffers, puis reboot explicite
      delay(200);
      ota_vlog("OTA rebooting");
      telemetry_log("OTA%20rebooting");
      ESP.restart();
      return true;
    case HTTP_UPDATE_NO_UPDATES:
      ota_vlog("OTA no updates");
      telemetry_log("OTA%20no%20updates");
      if (errMsg) *errMsg = "no updates";
      return false;
    case HTTP_UPDATE_FAILED: {
      String e = String("fail err=") + httpUpdate.getLastError() + " " + httpUpdate.getLastErrorString();
      ota_vlog(String("OTA update failed ") + e);
      telemetry_log((String("OTA%20fail%20") + ota_percent_encode(e)).c_str());
      if (errMsg) *errMsg = e;
      return false;
    }
  }
  return false;
}


