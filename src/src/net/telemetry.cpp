// telemetry.cpp — Native WebSocket server/client and JSON publishing
#include "telemetry.h"
#include "../config/Config.h"
#include "../common/timing.h"
#include "commands.h"
#include "mqtt_manager.h"
#include "wifi_manager.h"
#include <vector>
#include <string>
#include <stdarg.h>

extern "C" {
  #include "esp_http_server.h"
  #include "esp_log.h"
  #include "esp_event.h"
  #include "esp_websocket_client.h"
}

static const char* TAG = "telemetry";

// httpd server
static httpd_handle_t s_server = nullptr;
static std::vector<int> s_wsClients; // socket fds
static TelemetryNetMode s_mode = TelemetryNetMode::Server;

// websocket client
static esp_websocket_client_handle_t s_wsClient = nullptr;

// -------------- Utilities --------------
static std::string percent_encode_spaces(const char* msg) {
  std::string out;
  for (const char* p = msg; *p; ++p) {
    if (*p == ' ') {
      out += "%20"; // encode spaces as requested [[memory:8240683]]
    } else if (*p == '\n' || *p == '\r') {
      // skip newlines to ensure single line logs
    } else {
      out.push_back(*p);
    }
  }
  return out;
}

// -------------- HTTP root (viewer) --------------
static const char* HTML_VIEWER = R"HTML(<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Lexaro MLX90640 Viewer</title>
  <style>
    body { font-family: sans-serif; margin: 12px; background: #111; color: #eee; }
    #info { margin-bottom: 8px; }
    canvas { image-rendering: pixelated; border: 1px solid #444; background: #000; }
  </style>
</head>
<body>
  <div id="info">Connecting...</div>
  <canvas id="cv" width="320" height="240"></canvas>
  <script>
    const info = document.getElementById('info');
    const cv = document.getElementById('cv');
    const ctx = cv.getContext('2d');
    const scale = 10; // 32x24 -> 320x240
    function decodeRLE(str, w, h) {
      const out = new Float32Array(w*h);
      let idx = 0;
      const parts = str.split(';');
      for (let p of parts) {
        if (!p) continue;
        const kv = p.split(':');
        if (kv.length !== 2) continue;
        const cnt = parseInt(kv[0], 10);
        const val = parseInt(kv[1], 10) / 100.0;
        for (let i=0; i<cnt && idx < out.length; i++) out[idx++] = val;
      }
      return out;
    }
    function palette(v) {
      const x = Math.max(0, Math.min(1, v));
      const r = Math.floor(255 * Math.min(1, Math.max(0, 1.5*x - 0.5)));
      const g = Math.floor(255 * Math.min(1, Math.max(0, 1.5*(x-0.33))));
      const b = Math.floor(255 * Math.min(1, Math.max(0, 1.5*(0.66 - x))));
      return [r, g, b];
    }
    function renderFrame(w, h, temps) {
      const arr = temps;
      const sorted = Array.from(arr).sort((a,b)=>a-b);
      const lo = sorted[Math.floor(sorted.length*0.05)];
      const hi = sorted[Math.floor(sorted.length*0.95)];
      const img = ctx.createImageData(w, h);
      for (let i=0; i<arr.length; i++) {
        const t = (arr[i]-lo)/(hi-lo || 1);
        const [r,g,b] = palette(t);
        const j = i*4;
        img.data[j+0]=r; img.data[j+1]=g; img.data[j+2]=b; img.data[j+3]=255;
      }
      const tmp = document.createElement('canvas');
      tmp.width = w; tmp.height = h;
      const tctx = tmp.getContext('2d');
      tctx.putImageData(img, 0, 0);
      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(tmp, 0, 0, w*scale, h*scale);
    }
    function connectWS() {
      const proto = (location.protocol === 'https:') ? 'wss' : 'ws';
      const url = proto + '://' + location.host + '/ws';
      const ws = new WebSocket(url);
      ws.onopen = () => { info.textContent = 'WebSocket connecté: ' + url; };
      ws.onmessage = ev => {
        try {
          const msg = JSON.parse(ev.data);
          if (msg.type === 'ir' && msg.w && msg.h && msg.pix) {
            const arr = decodeRLE(msg.pix, msg.w, msg.h);
            renderFrame(msg.w, msg.h, arr);
            if (msg.spot) info.textContent = `IR ${msg.w}x${msg.h} spot T=${msg.spot.t}`;
          }
        } catch(e) {}
      };
      ws.onclose = () => { info.textContent = 'WS fermé, reconnexion...'; setTimeout(connectWS, 1000); };
      ws.onerror = () => { info.textContent = 'WS erreur'; };
    }
    connectWS();
  </script>
</body>
</html>)HTML";

static esp_err_t root_handler(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_send(req, HTML_VIEWER, HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}
static void ws_server_broadcast_text(const char* data, size_t len) {
  if (!s_server) return;
  httpd_ws_frame_t ws_pkt = {};
  ws_pkt.type = HTTPD_WS_TYPE_TEXT;
  ws_pkt.payload = (uint8_t*)data;
  ws_pkt.len = len;
  for (int sock : s_wsClients) {
    httpd_ws_send_frame_async(s_server, sock, &ws_pkt);
  }
}

void telemetry_stop_server() {
  if (s_server) {
    httpd_stop(s_server);
    s_server = nullptr;
    s_wsClients.clear();
    telemetry_log("WS%20server%20stopped");
  }
}

bool telemetry_is_server_running() {
  return s_server != nullptr;
}

static void publish_text(const char* data) {
  // Server-side broadcast
  ws_server_broadcast_text(data, strlen(data));
  // Client-side forward if connected
  if (s_wsClient && esp_websocket_client_is_connected(s_wsClient)) {
    esp_websocket_client_send_text(s_wsClient, data, strlen(data), portMAX_DELAY);
  }
}

// -------------- HTTPD WS handler --------------
static esp_err_t ws_handler(httpd_req_t* req) {
  if (req->method == HTTP_GET) {
    // New connection upgrade
    int sock = httpd_req_to_sockfd(req);
    bool known = false;
    for (int s : s_wsClients) if (s == sock) known = true;
    if (!known) s_wsClients.push_back(sock);
    telemetry_log("WS%20client%20connected");
    return ESP_OK;
  }

  httpd_ws_frame_t ws_pkt;
  memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
  ws_pkt.type = HTTPD_WS_TYPE_TEXT;
  // First, get frame length
  esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
  if (ret != ESP_OK) {
    return ret;
  }
  if (ws_pkt.len > 0) {
    std::vector<uint8_t> buf(ws_pkt.len + 1, 0);
    ws_pkt.payload = buf.data();
    ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
    if (ret == ESP_OK) {
      // Handle command line
      ((char*)ws_pkt.payload)[ws_pkt.len] = 0;
      commands_handle_line((const char*)ws_pkt.payload);
    }
  }
  return ESP_OK;
}

void telemetry_begin_server(const char* path, uint16_t port) {
  if (s_server) return;
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = port;
  config.lru_purge_enable = true;
  config.max_open_sockets = 7;

  if (httpd_start(&s_server, &config) == ESP_OK) {
    httpd_uri_t root = {};
    root.uri = "/";
    root.method = HTTP_GET;
    root.handler = root_handler;
    root.user_ctx = nullptr;
    root.is_websocket = false;
    httpd_register_uri_handler(s_server, &root);
    httpd_uri_t ws = {};
    ws.uri = path;
    ws.method = HTTP_GET;
    ws.handler = ws_handler;
    ws.user_ctx = nullptr;
    ws.is_websocket = true;
    httpd_register_uri_handler(s_server, &ws);
    telemetry_log("WS%20server%20started");
  }
}

void telemetry_set_mode(TelemetryNetMode mode) {
  s_mode = mode;
  if (mode == TelemetryNetMode::Server) {
    telemetry_log("Mode%20Server");
  } else {
    telemetry_log("Mode%20Client");
  }
}

static void ws_client_event(void* handler_args, esp_event_base_t base, int32_t event_id, void* event_data) {
  switch (event_id) {
    case WEBSOCKET_EVENT_CONNECTED:
      telemetry_log("WS%20client%20connected");
      break;
    case WEBSOCKET_EVENT_DISCONNECTED:
      telemetry_log("WS%20client%20disconnected");
      break;
    case WEBSOCKET_EVENT_DATA: {
      esp_websocket_event_data_t* data = (esp_websocket_event_data_t*)event_data;
      if (data->op_code == 1 && data->data_len > 0) {
        // Text frame
        std::string msg((const char*)data->data_ptr, data->data_len);
        commands_handle_line(msg.c_str());
      }
      break;
    }
    default: break;
  }
}

bool telemetry_begin_client(const char* url) {
  if (s_wsClient) {
    esp_websocket_client_destroy(s_wsClient);
    s_wsClient = nullptr;
  }
  esp_websocket_client_config_t cfg = {};
  cfg.uri = url;
  s_wsClient = esp_websocket_client_init(&cfg);
  if (!s_wsClient) return false;
  esp_websocket_register_events(s_wsClient, WEBSOCKET_EVENT_ANY, ws_client_event, nullptr);
  esp_err_t err = esp_websocket_client_start(s_wsClient);
  return err == ESP_OK;
}

void telemetry_loop() {
  // Housekeeping
  wifi_manager_loop();
  mqtt_loop();
}

// -------------- JSON helpers --------------
static void json_escape_and_append(String& s, const char* txt) {
  while (*txt) {
    char c = *txt++;
    if (c == '\"' || c == '\\') {
      s += '\\';
      s += c;
    } else if (c == '\n' || c == '\r') {
      // skip to keep single line
    } else {
      s += c;
    }
  }
}

void telemetry_publish_ir_frame(const IRFrame& frame, const IRSummary& summary) {
  String s;
  s.reserve(1024);
  s += "{\"type\":\"ir\",\"ts\":";
  s += (unsigned long long)frame.timestampUs;
  s += ",\"w\":"; s += frame.width;
  s += ",\"h\":"; s += frame.height;
  s += ",\"spot\":{";
  s += "\"x\":"; s += summary.valid ? String(summary.centroidX, 2) : "null";
  s += ",\"y\":"; s += summary.valid ? String(summary.centroidY, 2) : "null";
  s += ",\"t\":"; s += String(summary.tMaxC, 2);
  s += "}";
  // Append RLE-compressed heatmap (quantized *100, runs "count:val;" )
  s += ",\"pix\":\"";
  int16_t last = (int16_t)roundf(frame.pixelsC[0] * 100.0f);
  uint16_t run = 1;
  const int N = frame.width * frame.height;
  for (int i = 1; i < N; ++i) {
    int16_t q = (int16_t)roundf(frame.pixelsC[i] * 100.0f);
    if (q == last && run < 65535) {
      run++;
    } else {
      s += String(run); s += ":"; s += String(last); s += ";";
      last = q; run = 1;
    }
  }
  s += String(run); s += ":"; s += String(last); s += ";";
  s += "\"}";
  publish_text(s.c_str());
  // Publish a slim IR summary on MQTT (avoid large payloads/timeouts)
  String m;
  m.reserve(192);
  m += "{\"type\":\"ir_sum\",\"ts\":";
  m += (unsigned long long)frame.timestampUs;
  m += ",\"w\":"; m += frame.width;
  m += ",\"h\":"; m += frame.height;
  m += ",\"spot\":{";
  m += "\"x\":"; m += summary.valid ? String(summary.centroidX, 2) : "null";
  m += ",\"y\":"; m += summary.valid ? String(summary.centroidY, 2) : "null";
  m += ",\"t\":"; m += String(summary.tMaxC, 2);
  m += "}}";
  mqtt_publish_json("ir", m.c_str());
}

void telemetry_publish_radar(const RadarSample& sample) {
  String s;
  s.reserve(160);
  s += "{\"type\":\"radar\",\"ts\":";
  s += (unsigned long long)sample.timestampUs;
  s += ",\"presence\":"; s += (sample.presence ? "true" : "false");
  s += ",\"resp_bpm\":"; s += String(sample.respBpm, 1);
  s += ",\"motion\":{\"rms\":"; s += String(sample.motionRms, 3); s += "}}";
  publish_text(s.c_str());
  mqtt_publish_json("radar", s.c_str());
}

void telemetry_publish_fusion(const TracksView& view) {
  String s;
  s.reserve(256);
  s += "{\"type\":\"fusion\",\"ts\":";
  s += (unsigned long long)micros64();
  s += ",\"tracks\":[";
  for (size_t i = 0; i < view.count; ++i) {
    const Track& t = view.tracks[i];
    if (i) s += ",";
    s += "{\"id\":"; s += t.id;
    s += ",\"x\":"; s += String(t.x, 2);
    s += ",\"y\":"; s += String(t.y, 2);
    s += ",\"vx\":"; s += String(t.vx, 2);
    s += ",\"vy\":"; s += String(t.vy, 2);
    s += ",\"resp_bpm\":"; s += String(t.respBpm, 1);
    s += ",\"t_body\":"; s += String(t.tBody, 2);
    s += ",\"conf\":"; s += String(t.confidence, 2);
    s += "}";
  }
  s += "]}";
  publish_text(s.c_str());
  mqtt_publish_json("fusion", s.c_str());
}

void telemetry_publish_event(const char* kind, uint32_t trackId, float severity) {
  String s;
  s.reserve(160);
  s += "{\"type\":\"event\",\"ts\":";
  s += (unsigned long long)micros64();
  s += ",\"kind\":\""; s += kind; s += "\"";
  s += ",\"id\":"; s += trackId;
  s += ",\"severity\":"; s += String(severity, 2);
  s += "}";
  publish_text(s.c_str());
  mqtt_publish_json("event", s.c_str());
}

void telemetry_log(const char* msg) {
  std::string enc = percent_encode_spaces(msg); // enforce single-line and %20 for spaces [[memory:8240683]]
  publish_text(enc.c_str());
}

void telemetry_logf(const char* fmt, ...) {
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  telemetry_log(buf);
}


