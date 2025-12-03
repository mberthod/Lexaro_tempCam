// mqtt_manager.h — MQTT connection and publish helpers
#pragma once

#include <Arduino.h>

struct MqttConfig {
  String host;
  uint16_t port;
  String user;
  String pass;
  String clientId;
  String baseTopic; // e.g., "lexaro"
};

bool mqtt_set_config(const MqttConfig& cfg);
bool mqtt_connect();
void mqtt_enable(bool en);
bool mqtt_is_connected();
void mqtt_loop();
bool mqtt_publish_json(const char* suffix, const char* json); // publishes to baseTopic/suffix


