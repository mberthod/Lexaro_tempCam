// mqtt_manager.cpp — MQTT connection and publish helpers
#include "mqtt_manager.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <Preferences.h>

static WiFiClient s_wifiClient;
static PubSubClient s_mqtt(s_wifiClient);
static Preferences s_prefs;
static MqttConfig s_cfg;
static bool s_enabled = false;

bool mqtt_set_config(const MqttConfig& cfg) {
  s_cfg = cfg;
  s_prefs.begin("lexaro", false);
  s_prefs.putString("mqtt_host", s_cfg.host);
  s_prefs.putUShort("mqtt_port", s_cfg.port);
  s_prefs.putString("mqtt_user", s_cfg.user);
  s_prefs.putString("mqtt_pass", s_cfg.pass);
  s_prefs.putString("mqtt_base", s_cfg.baseTopic);
  if (s_cfg.clientId.length() == 0) {
    s_cfg.clientId = String("lexaro-") + String((uint32_t)ESP.getEfuseMac(), HEX);
  }
  s_mqtt.setServer(s_cfg.host.c_str(), s_cfg.port);
  // Robust params: larger buffer for JSON, longer keepalive
  s_mqtt.setKeepAlive(30);
  s_mqtt.setBufferSize(4096);
  return true;
}

bool mqtt_connect() {
  if (!s_enabled) return false;
  if (s_mqtt.connected()) return true;
  if (WiFi.status() != WL_CONNECTED) return false;
  if (s_cfg.clientId.length() == 0) {
    s_cfg.clientId = String("lexaro-") + String((uint32_t)ESP.getEfuseMac(), HEX);
  }
  bool ok = false;
  if (s_cfg.user.length() > 0) {
    ok = s_mqtt.connect(s_cfg.clientId.c_str(), s_cfg.user.c_str(), s_cfg.pass.c_str());
  } else {
    ok = s_mqtt.connect(s_cfg.clientId.c_str());
  }
  return ok;
}

void mqtt_enable(bool en) {
  s_enabled = en;
  if (en) {
    s_prefs.begin("lexaro", true);
    s_cfg.host = s_prefs.getString("mqtt_host", "");
    s_cfg.port = s_prefs.getUShort("mqtt_port", 1883);
    s_cfg.user = s_prefs.getString("mqtt_user", "");
    s_cfg.pass = s_prefs.getString("mqtt_pass", "");
    s_cfg.baseTopic = s_prefs.getString("mqtt_base", "lexaro");
    if (s_cfg.clientId.length() == 0) {
      s_cfg.clientId = String("lexaro-") + String((uint32_t)ESP.getEfuseMac(), HEX);
    }
    s_mqtt.setServer(s_cfg.host.c_str(), s_cfg.port);
  }
}

bool mqtt_is_connected() {
  return s_mqtt.connected();
}

void mqtt_loop() {
  if (!s_enabled) return;
  if (WiFi.status() != WL_CONNECTED) return;
  if (!s_mqtt.connected()) {
    mqtt_connect();
  }
  s_mqtt.loop();
}

bool mqtt_publish_json(const char* suffix, const char* json) {
  if (!s_enabled) return false;
  if (!s_mqtt.connected()) return false;
  String topic = s_cfg.baseTopic;
  if (!topic.endsWith("/")) topic += "/";
  topic += suffix;
  return s_mqtt.publish(topic.c_str(), json, true);
}


