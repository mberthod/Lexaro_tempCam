#include "ota_manager.h"
#include <Preferences.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include "telemetry.h"

static Preferences s_prefs;
static OtaConfig s_cfg;

bool ota_init() {
  s_prefs.begin("lexaro", false);
  s_cfg.url = s_prefs.getString("ota_url", "");
  s_cfg.autoCheck = s_prefs.getBool("ota_auto", false);
  if (s_cfg.url.length() == 0) {
    // URL GitHub par défaut (releases/latest) pour Lexaro_tempCam
    s_cfg.url = "https://github.com/mberthod/Lexaro_tempCam/releases/latest/download/firmware.bin";
  }
  return true;
}

void ota_set_url(const String& url) {
  s_cfg.url = url;
  s_prefs.putString("ota_url", url);
}

String ota_get_url() {
  return s_cfg.url;
}

void ota_set_auto(bool en) {
  s_cfg.autoCheck = en;
  s_prefs.putBool("ota_auto", en);
}

bool ota_get_auto() {
  return s_cfg.autoCheck;
}

String ota_status() {
  String s = "OTA url=" + (s_cfg.url.length() ? s_cfg.url : String("<none>"));
  s += " auto="; s += (s_cfg.autoCheck ? "on" : "off");
  return s;
}

bool ota_run_now(String* errMsg) {
  if (s_cfg.url.length() == 0) {
    if (errMsg) *errMsg = "empty url";
    telemetry_log("ERR%20ota%20empty%20url");
    return false;
  }
  if (WiFi.status() != WL_CONNECTED) {
    if (errMsg) *errMsg = "wifi disconnected";
    telemetry_log("ERR%20ota%20wifi");
    return false;
  }
  telemetry_log("OTA%20start");
  WiFiClientSecure client;
  client.setInsecure(); // GitHub TLS sans validation de cert (simple et robuste)
  httpUpdate.rebootOnUpdate(true);
  t_httpUpdate_return ret = httpUpdate.update(client, s_cfg.url);
  switch (ret) {
    case HTTP_UPDATE_FAILED: {
      String e = String("OTA fail err=") + httpUpdate.getLastError() + " " + httpUpdate.getLastErrorString();
      telemetry_log(e.c_str());
      if (errMsg) *errMsg = e;
      return false;
    }
    case HTTP_UPDATE_NO_UPDATES:
      telemetry_log("OTA%20no%20updates");
      if (errMsg) *errMsg = "no updates";
      return false;
    case HTTP_UPDATE_OK:
      telemetry_log("OTA%20ok%20reboot");
      // reboot automatique (rebootOnUpdate=true)
      return true;
  }
  return false;
}


