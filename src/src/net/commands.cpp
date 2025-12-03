// commands.cpp — parser for percent-encoded monoline commands
#include "commands.h"
#include "telemetry.h"
#include "wifi_manager.h"
#include "mqtt_manager.h"
#include "ota_manager.h"
#include <Preferences.h>
#include <string>
#include <vector>
#include <ctype.h>

static SetU16Func s_setIrFps;
static SetU16Func s_setRadarFps;
static SetModeFunc s_setMode;

static std::string percent_decode(const char* in) {
  std::string out;
  for (size_t i = 0; in[i]; ++i) {
    if (in[i] == '%' && isxdigit(in[i+1]) && isxdigit(in[i+2])) {
      char hex[3] = { in[i+1], in[i+2], 0 };
      char c = (char) strtol(hex, nullptr, 16);
      out.push_back(c);
      i += 2;
    } else if (in[i] == '+') {
      out.push_back(' ');
    } else if (in[i] == '\n' || in[i] == '\r') {
      // skip
    } else {
      out.push_back(in[i]);
    }
  }
  return out;
}

static void handle_set_query(const std::string& query) {
  // parse key=value pairs separated by &
  size_t pos = 0;
  while (pos < query.size()) {
    size_t amp = query.find('&', pos);
    std::string pair = query.substr(pos, amp == std::string::npos ? std::string::npos : amp - pos);
    size_t eq = pair.find('=');
    if (eq != std::string::npos) {
      std::string key = pair.substr(0, eq);
      std::string val = pair.substr(eq + 1);
      if (key == "ir_fps" && s_setIrFps) {
        s_setIrFps((uint16_t)atoi(val.c_str()));
      } else if (key == "radar_fps" && s_setRadarFps) {
        s_setRadarFps((uint16_t)atoi(val.c_str()));
      }
    }
    if (amp == std::string::npos) break;
    pos = amp + 1;
  }
  telemetry_log("ACK%20set");
}

void commands_bind_setters(SetU16Func setIrFps, SetU16Func setRadarFps, SetModeFunc setMode) {
  s_setIrFps = setIrFps;
  s_setRadarFps = setRadarFps;
  s_setMode = setMode;
}

void commands_handle_line(const char* line) {
  std::string s = percent_decode(line);
  // Make tolerant checks for some commands (trim spaces)
  auto trim = [](std::string& str){
    size_t a = str.find_first_not_of(" \t\r\n");
    size_t b = str.find_last_not_of(" \t\r\n");
    if (a == std::string::npos) { str.clear(); return; }
    str = str.substr(a, b - a + 1);
  };
  trim(s);
  if (s.rfind("/set", 0) == 0) {
    size_t q = s.find('?');
    if (q != std::string::npos) {
      handle_set_query(s.substr(q + 1));
    } else {
      telemetry_log("ERR%20set%20missing%20query");
    }
  } else if (s.rfind("/calibrate temp", 0) == 0) {
    telemetry_log("ACK%20calibrate%20temp");
  } else if (s.rfind("/net mode=", 0) == 0) {
    std::string mode = s.substr(strlen("/net mode="));
    if (mode == "server") {
      if (s_setMode) s_setMode(TelemetryNetMode::Server);
      telemetry_log("ACK%20net%20server");
    } else if (mode == "client") {
      if (s_setMode) s_setMode(TelemetryNetMode::Client);
      telemetry_log("ACK%20net%20client");
    } else {
      telemetry_log("ERR%20net%20mode");
    }
  } else if (s.rfind("/ws target=", 0) == 0) {
    std::string url = s.substr(strlen("/ws target="));
    if (telemetry_begin_client(url.c_str())) {
      telemetry_log("ACK%20ws%20client%20start");
    } else {
      telemetry_log("ERR%20ws%20client");
    }
  } else if (s.rfind("/wifi set?", 0) == 0) {
    size_t q = s.find('?');
    std::string query = (q != std::string::npos) ? s.substr(q + 1) : "";
    std::string ssid, pass;
    size_t pos = 0;
    while (pos < query.size()) {
      size_t amp = query.find('&', pos);
      std::string pair = query.substr(pos, amp == std::string::npos ? std::string::npos : amp - pos);
      size_t eq = pair.find('=');
      if (eq != std::string::npos) {
        std::string key = pair.substr(0, eq);
        std::string val = pair.substr(eq + 1);
        if (key == "ssid") ssid = val;
        else if (key == "pass") pass = val;
      }
      if (amp == std::string::npos) break;
      pos = amp + 1;
    }
    if (!ssid.empty()) {
      bool ok = wifi_manager_connect(ssid.c_str(), pass.c_str(), true);
      telemetry_log(ok ? "ACK%20wifi%20connect" : "ERR%20wifi%20connect");
    } else {
      telemetry_log("ERR%20wifi%20missing%20ssid");
    }
  } else if (s == "/wifi portal" || s.find("/wifi portal ") == 0) {
    telemetry_stop_server(); // free port 80 for captive portal
    bool ok = wifi_manager_start_portal(nullptr, nullptr, 300000); // 5 min
    telemetry_log(ok ? "ACK%20wifi%20portal%20start" : "ERR%20wifi%20portal");
  } else if (s == "/wifi portal stop") {
    wifi_manager_stop_portal();
    telemetry_begin_server(LEXARO_WS_PATH, LEXARO_HTTP_PORT); // restore WS server
    telemetry_log("ACK%20wifi%20portal%20stop");
  } else if (s == "/wifi portal status") {
    bool run = wifi_manager_portal_running();
    telemetry_log(run ? "ACK%20wifi%20portal%20running" : "ACK%20wifi%20portal%20stopped");
  } else if (s == "/ws restart") {
    telemetry_stop_server();
    telemetry_begin_server(LEXARO_WS_PATH, LEXARO_HTTP_PORT);
    telemetry_log("ACK%20ws%20restart");
  } else if (s == "/ws stop") {
    telemetry_stop_server();
    telemetry_log("ACK%20ws%20stopped");
  } else if (s == "/ws start") {
    telemetry_begin_server(LEXARO_WS_PATH, LEXARO_HTTP_PORT);
    telemetry_log("ACK%20ws%20started");
  } else if (s.rfind("/mqtt set?", 0) == 0) {
    size_t q = s.find('?');
    std::string query = (q != std::string::npos) ? s.substr(q + 1) : "";
    MqttConfig cfg;
    cfg.port = 1883;
    cfg.baseTopic = "lexaro";
    size_t pos = 0;
    while (pos < query.size()) {
      size_t amp = query.find('&', pos);
      std::string pair = query.substr(pos, amp == std::string::npos ? std::string::npos : amp - pos);
      size_t eq = pair.find('=');
      if (eq != std::string::npos) {
        std::string key = pair.substr(0, eq);
        std::string val = pair.substr(eq + 1);
        if (key == "host") cfg.host = val.c_str();
        else if (key == "port") cfg.port = (uint16_t)atoi(val.c_str());
        else if (key == "user") cfg.user = val.c_str();
        else if (key == "pass") cfg.pass = val.c_str();
        else if (key == "topic") cfg.baseTopic = val.c_str();
      }
      if (amp == std::string::npos) break;
      pos = amp + 1;
    }
    mqtt_set_config(cfg);
    telemetry_log("ACK%20mqtt%20set");
  } else if (s == "/mqtt on") {
    mqtt_enable(true);
    telemetry_log("ACK%20mqtt%20on");
  } else if (s == "/mqtt off") {
    mqtt_enable(false);
    telemetry_log("ACK%20mqtt%20off");
  } else if (s == "/mqtt connect") {
    bool ok = mqtt_connect();
    telemetry_log(ok ? "ACK%20mqtt%20connect" : "ERR%20mqtt%20connect");
  } else if (s == "/mqtt status") {
    bool conn = mqtt_is_connected();
    Preferences prefs; prefs.begin("lexaro", true);
    String host = prefs.getString("mqtt_host", "");
    uint16_t port = prefs.getUShort("mqtt_port", 1883);
    String base = prefs.getString("mqtt_base", "lexaro");
    // Encode spaces as %20 (telemetry_log already does this). Avoid spaces in values.
    String msg = String("ACK mqtt status conn=") + (conn ? "yes" : "no")
      + " host=" + host + " port=" + String(port) + " topic=" + base;
    telemetry_log(msg.c_str());
  } else if (s == "/mqtt test") {
    const char* payload = "{\"type\":\"test\",\"ok\":true}";
    bool ok = mqtt_publish_json("test", payload);
    telemetry_log(ok ? "ACK%20mqtt%20test" : "ERR%20mqtt%20test");
  } else if (s.rfind("/ota set?", 0) == 0) {
    size_t q = s.find('?');
    std::string query = (q != std::string::npos) ? s.substr(q + 1) : "";
    std::string url;
    size_t pos = 0;
    while (pos < query.size()) {
      size_t amp = query.find('&', pos);
      std::string pair = query.substr(pos, amp == std::string::npos ? std::string::npos : amp - pos);
      size_t eq = pair.find('=');
      if (eq != std::string::npos) {
        std::string key = pair.substr(0, eq);
        std::string val = pair.substr(eq + 1);
        if (key == "url") url = val;
      }
      if (amp == std::string::npos) break;
      pos = amp + 1;
    }
    if (url.empty()) { telemetry_log("ERR%20ota%20missing%20url"); }
    else { ota_set_url(url.c_str()); telemetry_log("ACK%20ota%20set"); }
  } else if (s == "/ota run") {
    String err;
    bool ok = ota_run_now(&err);
    if (!ok) {
      String m = String("ERR ota ") + err;
      telemetry_log(m.c_str());
    }
  } else if (s == "/ota status") {
    String st = ota_status();
    telemetry_log(st.c_str());
  } else if (s == "/ota auto on") {
    ota_set_auto(true);
    telemetry_log("ACK%20ota%20auto%20on");
  } else if (s == "/ota auto off") {
    ota_set_auto(false);
    telemetry_log("ACK%20ota%20auto%20off");
  } else if (s == "/help") {
    telemetry_log("ACK%20help%20use%20/ws%20restart%7Cstop%7Cstart%2C%20/wifi%20portal%7Cportal%20stop%2C%20/mqtt%20set%7Con%7Cconnect%7Cstatus");
  } else {
    std::string msg = "ERR unknown cmd=" + s;
    telemetry_log(msg.c_str());
  }
}


