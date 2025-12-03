// Lexaro TempCam + Radar — Arduino (ESP32‑S3) skeleton
// Framework: Arduino-ESP32 v3.x (ESP-IDF 5.x)
// WebSocket natif (serveur httpd + client esp_websocket_client), logs monoligne percent-encodées

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <HardwareSerial.h>
#include "driver/i2s.h"
#include "driver/gpio.h"
#include <Preferences.h>

// -------- Firmware metadata --------
#define LEXARO_FW_NAME      "Lexaro TempCam"
#define LEXARO_FW_VERSION   "0.0.2"
#define LEXARO_FW_AUTHOR    "Mathieu / Hexacore"
#define LEXARO_FW_COMPANY   "Lexaro"
#define LEXARO_FW_BUILD_DATE __DATE__ " " __TIME__

// IDF headers (C linkage)
extern "C" {
  #include "esp_timer.h"
  #include "esp_system.h"
  #include "esp_wifi.h"
}

#include "src/config/Config.h"
#include "src/common/timing.h"
#include "src/common/ring_buffer.h"
#include "src/drivers/tmp117_driver.h"
#include "src/drivers/ir_manager.h"
#include "src/drivers/radar_manager.h"
#include "src/core/calibration.h"
#include "src/core/filters.h"
#include "src/core/fusion_engine.h"
#include "src/net/telemetry.h"
#include "src/net/commands.h"
#include "src/net/wifi_manager.h"
#include "src/net/mqtt_manager.h"
#include "src/net/ota_manager.h"
#include "src/ui/ui_status.h"

// -----------------------------
// Globals (device singletons)
// -----------------------------
TwoWire g_i2c = TwoWire(0);
TwoWire g_i2cSensor = TwoWire(1);
SPIClass g_spi(FSPI);
HardwareSerial RadarUART(1);

TMP117Driver g_tmp117;
IRManager g_irManager;
RadarManager g_radarManager;
FusionEngine g_fusion;


// Runtime configuration (modifiable via /set commands)
volatile uint16_t g_irFps = 8;
volatile uint16_t g_radarFps = 30;

// Inter-task channels
static constexpr size_t IR_QUEUE_CAPACITY = 8;
static constexpr size_t RADAR_QUEUE_CAPACITY = 32;
static RingBuffer<IRSample, IR_QUEUE_CAPACITY> g_irQueue;
static RingBuffer<RadarSample, RADAR_QUEUE_CAPACITY> g_radarQueue;

// Telemetry control
volatile bool g_publishIR = true;
volatile bool g_publishRadar = true;
volatile bool g_publishFusion = true;

// -----------------------------
// Task declarations
// -----------------------------
static TaskHandle_t s_taskIR = nullptr;
static TaskHandle_t s_taskRadar = nullptr;
static TaskHandle_t s_taskFusion = nullptr;
static TaskHandle_t s_taskTelemetry = nullptr;
static TaskHandle_t s_taskUI = nullptr;
static TaskHandle_t s_taskUartBridge = nullptr;
static TaskHandle_t s_taskSensors = nullptr;
static TaskHandle_t s_taskMic = nullptr;

void taskIR(void* param);
void taskRadar(void* param);
void taskFusion(void* param);
void taskTelemetry(void* param);
void taskUI(void* param);
void taskUartBridge(void* param);
void taskSensors(void* param);
void taskMic(void* param);

// -------- LD6002H decoded shared state --------
static volatile float g_decodedRespBpm = NAN;
static volatile float g_decodedHeartBpm = NAN;
static volatile uint16_t g_decodedDistCm = 0;
static volatile uint8_t g_decodedPresence = 0;
static volatile uint8_t g_decodedMotion = 0;
static volatile uint8_t g_decodedBucket = 0;
static volatile uint64_t g_decodedTsUs = 0;

// -------- Verbosity flags --------
static volatile bool g_verboseSensors = false; // default: silent
static volatile bool g_verboseMlx = false;

// -----------------------------
// Helpers
// -----------------------------
static void print_boot_banner() {
  Serial.println();
  Serial.println("======================================");
  Serial.printf("%s v%s\r\n", LEXARO_FW_NAME, LEXARO_FW_VERSION);
  Serial.printf("Build: %s\r\n", LEXARO_FW_BUILD_DATE);
  Serial.printf("Author: %s  Company: %s\r\n", LEXARO_FW_AUTHOR, LEXARO_FW_COMPANY);
  Serial.printf("ChipID: %llX  CPU: ESP32-S3\r\n", (unsigned long long)ESP.getEfuseMac());
  Serial.println("--------------------------------------");
  Serial.println("Base commands:");
  Serial.println("  help");
  Serial.println("  wifi status | wifi connect ssid=<ssid> pass=<pass> save=1");
  Serial.println("  mqtt status | mqtt on|off | mqtt connect | mqtt set host=<h> port=<p> user=<u> pass=<p> topic=<t>");
  Serial.println("  sensors status | sensors verbose on|off|status");
  Serial.println("  mlx status | system info");
  Serial.println("======================================");
}

// -------- MLX90640 debug helpers --------
static int i2c_read16_debug(TwoWire& w, uint8_t addr, uint16_t reg, uint16_t& out) {
  unsigned long t0 = millis();
  int rc;
  w.beginTransmission(addr);
  w.write((uint8_t)((reg >> 8) & 0xFF));
  w.write((uint8_t)(reg & 0xFF));
  rc = w.endTransmission(false); // repeated start
  if (g_verboseMlx) Serial.printf("[MLX] TX addr=0x%02X reg=0x%04X endTx=%d dt=%lums\r\n", addr, reg, rc, millis()-t0);
  if (rc != 0) return rc;
  int n = w.requestFrom((int)addr, 2, (int)true);
  if (g_verboseMlx) Serial.printf("[MLX] RX req=2 got=%d\r\n", n);
  if (n != 2) return -1;
  uint8_t msb = w.read();
  uint8_t lsb = w.read();
  out = ((uint16_t)msb << 8) | lsb;
  if (g_verboseMlx) Serial.printf("[MLX] DATA 0x%02X 0x%02X => 0x%04X\r\n", msb, lsb, out);
  return 0;
}

static void mlx_probe_debug() {
  const uint8_t addr = MLX90640_I2C_ADDR;
  uint16_t v;
  Serial.println("[MLX] Probe start");
  int r1 = i2c_read16_debug(g_i2c, addr, 0x8000, v); // status register
  Serial.printf("[MLX] Read 0x8000 rc=%d val=0x%04X\r\n", r1, v);
  int r2 = i2c_read16_debug(g_i2c, addr, 0x800D, v); // control register1
  Serial.printf("[MLX] Read 0x800D rc=%d val=0x%04X\r\n", r2, v);
}

static void setupI2C() {
  // Enable internal pull-ups on both I2C buses (weak, mais utile en debug)
  gpio_pulldown_dis((gpio_num_t)LEXAROSENSOR_I2C_SCL);
  gpio_pulldown_dis((gpio_num_t)LEXAROSENSOR_I2C_SDA);
  gpio_pullup_en((gpio_num_t)LEXAROSENSOR_I2C_SCL);
  gpio_pullup_en((gpio_num_t)LEXAROSENSOR_I2C_SDA);
  gpio_pulldown_dis((gpio_num_t)LEXARO_I2C_SCL);
  gpio_pulldown_dis((gpio_num_t)LEXARO_I2C_SDA);
  gpio_pullup_en((gpio_num_t)LEXARO_I2C_SCL);
  gpio_pullup_en((gpio_num_t)LEXARO_I2C_SDA);

  g_i2cSensor.begin(LEXAROSENSOR_I2C_SDA, LEXAROSENSOR_I2C_SCL, 100000);
  g_i2c.begin(LEXARO_I2C_SDA, LEXARO_I2C_SCL, 1000000);
  g_i2c.setClock(1000000);
  g_i2c.setTimeOut(50); // éviter blocages I2C
}

static void i2c_scan(TwoWire& wire, const char* tag) {
  Serial.printf("I2C scan on %s:\r\n", tag);
  for (uint8_t addr = 0x08; addr < 0x78; ++addr) {
    wire.beginTransmission(addr);
    if (wire.endTransmission() == 0) {
      Serial.printf("  - 0x%02X\r\n", addr);
    }
    delay(2);
  }
}

static void setupMic() {
  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
  cfg.sample_rate = 16000;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT; // capture L/R interleaved (compat plus large)
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags = 0;
  cfg.dma_buf_count = 4;
  cfg.dma_buf_len = 256;
  cfg.use_apll = false;
  cfg.tx_desc_auto_clear = false;
  cfg.fixed_mclk = 0;
  i2s_driver_install(I2S_NUM_0, &cfg, 0, nullptr);
  i2s_pin_config_t pins = {};
  pins.bck_io_num = I2S_MIC_SCK;
  pins.ws_io_num = I2S_MIC_WS;
  pins.data_out_num = I2S_PIN_NO_CHANGE;
  pins.data_in_num = I2S_MIC_SD;
  pins.mck_io_num = I2S_PIN_NO_CHANGE;
  i2s_set_pin(I2S_NUM_0, &pins);
  i2s_zero_dma_buffer(I2S_NUM_0);
}

static uint8_t bcd2bin(uint8_t v) { return ((v >> 4) * 10) + (v & 0x0F); }

static bool ds3231_read_tm(TwoWire& wire, int& year, int& month, int& day, int& hour, int& minute, int& second, float& tempC) {
  wire.beginTransmission(DS3231_ADDR);
  wire.write(0x00);
  if (wire.endTransmission(false) != 0) return false;
  if (wire.requestFrom((int)DS3231_ADDR, 7) != 7) return false;
  uint8_t sec = wire.read();
  uint8_t min = wire.read();
  uint8_t hrs = wire.read();
  wire.read(); // weekday
  uint8_t date = wire.read();
  uint8_t mon = wire.read();
  uint8_t yr = wire.read();
  second = bcd2bin(sec & 0x7F);
  minute = bcd2bin(min & 0x7F);
  hour = bcd2bin(hrs & 0x3F);
  day = bcd2bin(date);
  month = bcd2bin(mon & 0x1F);
  year = 2000 + bcd2bin(yr);
  // Temperature (0x11 MSB, 0x12 LSB upper bits)
  wire.beginTransmission(DS3231_ADDR);
  wire.write(0x11);
  if (wire.endTransmission(false) != 0) return true; // time ok, temp unknown
  if (wire.requestFrom((int)DS3231_ADDR, 2) == 2) {
    int8_t msb = (int8_t)wire.read();
    uint8_t lsb = wire.read();
    tempC = (float)msb + ((lsb >> 6) * 0.25f);
  }
  return true;
}

static bool hdc1080_read(TwoWire& wire, float& tempC, float& humPct) {
  // Temperature
  wire.beginTransmission(HDC1080_ADDR);
  wire.write(0x00);
  if (wire.endTransmission() != 0) return false;
  delay(20);
  if (wire.requestFrom((int)HDC1080_ADDR, 2) != 2) return false;
  uint16_t tRaw = ((uint16_t)wire.read() << 8) | wire.read();
  tempC = (tRaw / 65536.0f) * 165.0f - 40.0f;
  // Humidity
  wire.beginTransmission(HDC1080_ADDR);
  wire.write(0x01);
  if (wire.endTransmission() != 0) return false;
  delay(20);
  if (wire.requestFrom((int)HDC1080_ADDR, 2) != 2) return false;
  uint16_t hRaw = ((uint16_t)wire.read() << 8) | wire.read();
  humPct = (hRaw / 65536.0f) * 100.0f;
  return true;
}

static bool hdc1080_init(TwoWire& wire) {
  // Configure 14-bit T & RH, heater off
  wire.beginTransmission(HDC1080_ADDR);
  wire.write(0x02);
  wire.write(0x10);
  wire.write(0x00);
  return wire.endTransmission() == 0;
}

static bool veml6030_init_once(TwoWire& wire, uint8_t& addr) {
  static bool done = false;
  static uint8_t cached = 0;
  if (done) { addr = cached; return cached != 0; }
  uint8_t candidates[2] = { VEML6030_ADDR_A, VEML6030_ADDR_B };
  for (int i = 0; i < 2; ++i) {
    wire.beginTransmission(candidates[i]);
    if (wire.endTransmission() == 0) {
      // Configure ALS: default gain/IT, power on (write 0x0000 to 0x00)
      wire.beginTransmission(candidates[i]);
      wire.write(0x00);
      wire.write(0x00);
      wire.write(0x00);
      if (wire.endTransmission() == 0) {
        cached = candidates[i];
        done = true;
        addr = cached;
        return true;
      }
    }
  }
  done = true; cached = 0; addr = 0; return false;
}

static bool veml6030_read(TwoWire& wire, float& lux, uint16_t& rawAls) {
  uint8_t addr = 0;
  if (!veml6030_init_once(wire, addr)) return false;
  wire.beginTransmission(addr);
  wire.write(0x04);
  if (wire.endTransmission(false) != 0) return false;
  if (wire.requestFrom((int)addr, 2) != 2) return false;
  uint16_t als = (uint16_t)wire.read();
  als |= (uint16_t)wire.read() << 8;
  rawAls = als;
  // Approximate lux (depends on gain/IT; with default, scale ~0.0576)
  lux = als * 0.0576f;
  return true;
}

static void setupPower() {
  pinMode(POWER_CAM_PIN, OUTPUT);
//  pinMode(POWER_SD_PIN, OUTPUT);
  pinMode(POWER_MIC_PIN, OUTPUT);
  pinMode(POWER_VEML_PIN, OUTPUT);
  pinMode(POWER_SHT_PIN, OUTPUT);
  pinMode(POWER_MLX_PIN, OUTPUT);
  pinMode(POWER_RADAR_PIN, OUTPUT);
}

static void powerOn() {
  digitalWrite(POWER_CAM_PIN, HIGH);
  digitalWrite(POWER_MIC_PIN, HIGH);
  digitalWrite(POWER_VEML_PIN, HIGH);
  digitalWrite(POWER_SHT_PIN, HIGH);
  digitalWrite(POWER_MLX_PIN, HIGH);
  digitalWrite(POWER_RADAR_PIN, HIGH);
}
static void powerOff() {
  digitalWrite(POWER_CAM_PIN, LOW);
  //digitalWrite(POWER_SD_PIN, LOW);
  digitalWrite(POWER_MIC_PIN, LOW);
  digitalWrite(POWER_VEML_PIN, LOW);
  digitalWrite(POWER_SHT_PIN, LOW);
  digitalWrite(POWER_MLX_PIN, LOW);
  digitalWrite(POWER_RADAR_PIN, LOW);
}
static void setupWiFi() {
  // Start SoftAP as default; client mode can be enabled via WS command later
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(LEXARO_AP_SSID, LEXARO_AP_PASS);
}

static void startTasks() {
  xTaskCreatePinnedToCore(taskIR, "TaskIR", LEXARO_STACK_IR, nullptr, LEXARO_PRIO_IR, &s_taskIR, APP_CPU_NUM);
  xTaskCreatePinnedToCore(taskRadar, "TaskRadar", LEXARO_STACK_RADAR, nullptr, LEXARO_PRIO_RADAR, &s_taskRadar, PRO_CPU_NUM);
  xTaskCreatePinnedToCore(taskFusion, "TaskFusion", LEXARO_STACK_FUSION, nullptr, LEXARO_PRIO_FUSION, &s_taskFusion, PRO_CPU_NUM);
  xTaskCreatePinnedToCore(taskTelemetry, "TaskTelemetry", LEXARO_STACK_TELEM, nullptr, LEXARO_PRIO_TELEM, &s_taskTelemetry, APP_CPU_NUM);
  xTaskCreatePinnedToCore(taskUI, "TaskUI", LEXARO_STACK_UI, nullptr, LEXARO_PRIO_UI, &s_taskUI, APP_CPU_NUM);
}

// -----------------------------
// Arduino entry points
// -----------------------------
void setup() {
  Serial.begin(115200);
  delay(5000);
  
  setupPower();
  powerOn();
  delay(1000);

  setupI2C();
  if (g_verboseSensors) i2c_scan(g_i2cSensor, "SENSOR(47/48)");
  if (g_verboseSensors) i2c_scan(g_i2c, "MLX(9/10)");
  if (g_verboseSensors || g_verboseMlx) mlx_probe_debug();
  setupMic();
  // Init HDC1080 (if present)
  if (g_verboseSensors) {
    if (hdc1080_init(g_i2cSensor)) {
      Serial.println("HDC1080 init OK");
    } else {
      Serial.println("HDC1080 init FAIL");
    }
  } else {
    hdc1080_init(g_i2cSensor);
  }
  // Init MLX90640 on I2C (9/10)
  if (g_irManager.begin(g_i2c)) {
     Serial.println("MLX90640 init OK");
    // Start IR task if not already started
    xTaskCreatePinnedToCore(taskIR, "TaskIR", LEXARO_STACK_IR, nullptr, LEXARO_PRIO_IR, &s_taskIR, APP_CPU_NUM);
  } else {
    Serial.println("MLX90640 init FAIL (fallback synthetic)");
    xTaskCreatePinnedToCore(taskIR, "TaskIR", LEXARO_STACK_IR, nullptr, LEXARO_PRIO_IR, &s_taskIR, APP_CPU_NUM);
  }
  // Try Wi‑Fi STA from saved credentials (AP remains active)
  bool sta = wifi_manager_begin();
  // Boot banner with system info and basic commands
  print_boot_banner();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("WiFi connected: %s  IP: %s  RSSI: %d\r\n",
      WiFi.SSID().c_str(), WiFi.localIP().toString().c_str(), WiFi.RSSI());
  } else {
    Serial.println("WiFi not connected");
  }
  // Prepare MQTT (disabled by default; enable via WS cmd)
  MqttConfig mqc;
  mqc.host = "";
  mqc.port = 1883;
  mqc.user = "";
  mqc.pass = "";
  mqc.baseTopic = "lexaro";
  mqtt_set_config(mqc);
  mqtt_enable(false);
  // OTA: init et auto-check (si configuré et Wi‑Fi connecté)
  ota_init();
  if (ota_get_auto() && WiFi.status() == WL_CONNECTED) {
    telemetry_log("OTA%20auto%20check");
    String err;
    ota_run_now(&err); // si update dispo, la carte redémarre
  }
  
  // LD600CH radar UART on GPIO43 (RX) / GPIO44 (TX), 115200 8N1
  RadarUART.begin(115200, SERIAL_8N1, 43, 44);
  xTaskCreatePinnedToCore(taskUartBridge, "UartBridge", 4096, nullptr, 3, &s_taskUartBridge, APP_CPU_NUM);
  // Augmente la pile pour éviter overflow lors de flux soutenus / OTA
  vTaskDelete(s_taskUartBridge);
  xTaskCreatePinnedToCore(taskUartBridge, "UartBridge", 8192, nullptr, 3, &s_taskUartBridge, APP_CPU_NUM);
  xTaskCreatePinnedToCore(taskSensors, "EnvSensors", 4096, nullptr, 3, &s_taskSensors, APP_CPU_NUM);
  xTaskCreatePinnedToCore(taskMic, "MicRms", 4096, nullptr, 3, &s_taskMic, APP_CPU_NUM);
}

void loop() {
  // Idle; all work in tasks
  delay(1000);
}

// -----------------------------
// Task implementations
// -----------------------------
void taskIR(void* param) {
  const TickType_t minDelayTicks = pdMS_TO_TICKS(5);
  while (true) {
    const uint64_t ts = micros64();

    float ambientC = NAN;
    g_tmp117.readTemperatureC(ambientC);

    IRFrame frame;
    frame.timestampUs = ts;
    frame.width = IR_WIDTH;
    frame.height = IR_HEIGHT;
    if (!g_irManager.captureFrame(frame)) {
      // Provide fallback synthetic frame if sensor not ready
      g_irManager.syntheticFrame(frame, ambientC);
    }

    // Calibration using ambient from TMP117
    calibration_apply_ambient(frame, ambientC);

    // Simple segmentation and spot measurement
    IRSample sample;
    if (!g_irManager.extractSummary(frame, sample.summary)) {
      sample.summary.valid = false;
    }
    sample.timestampUs = frame.timestampUs;
    sample.ambientC = ambientC;
    sample.width = frame.width;
    sample.height = frame.height;
    memcpy(sample.centerValuesC, frame.centerValuesC, sizeof(sample.centerValuesC));

    g_irQueue.push(sample);

    if (g_publishIR) {
      telemetry_publish_ir_frame(frame, sample.summary);
    }

    const uint16_t fps = g_irFps;
    const uint32_t periodMs = max<uint32_t>(1000u / max<uint16_t>(fps, 1), 1u);
    vTaskDelay(max<TickType_t>(pdMS_TO_TICKS(periodMs), minDelayTicks));
  }
}

void taskRadar(void* param) {
  RespirationIIRFilter respFilter;
  respFilter.configureDefault();

  RespirationEstimator respEstimator;
  respEstimator.reset();

  const TickType_t minDelayTicks = pdMS_TO_TICKS(2);
  while (true) {
    const uint64_t ts = micros64();
    RadarRaw raw;
    if (!g_radarManager.sampleRaw(raw)) {
      g_radarManager.syntheticRaw(raw, ts);
    }

    float filtered = respFilter.process(raw.iComponent, raw.qComponent);
    // Distance/angle compensation (unknown -> assume 1.0m, 0 rad)
    filtered = compensate_distance_angle(filtered, 1.0f, 0.0f);
    const float bpm = respEstimator.update(ts, filtered);

    RadarSample sample;
    sample.timestampUs = ts;
    sample.filtered = filtered;
    // Use decoded presence if fresh (<= 2s), else fallback to raw
    if ((ts - g_decodedTsUs) <= 2000000ULL) {
      sample.presence = (g_decodedPresence != 0);
    } else {
      sample.presence = raw.presence;
    }
    sample.motionRms = raw.motionRms;
    // Prefer decoded respiration if available
    if ((ts - g_decodedTsUs) <= 2000000ULL && !isnan(g_decodedRespBpm)) {
      sample.respBpm = g_decodedRespBpm;
    } else {
      sample.respBpm = bpm;
    }
    g_radarQueue.push(sample);

    if (g_publishRadar) {
      telemetry_publish_radar(sample);
    }

    const uint16_t fps = g_radarFps;
    const uint32_t periodMs = max<uint32_t>(1000u / max<uint16_t>(fps, 1), 1u);
    vTaskDelay(max<TickType_t>(pdMS_TO_TICKS(periodMs), minDelayTicks));
  }
}

void taskFusion(void* param) {
  const TickType_t tick = pdMS_TO_TICKS(50);
  IRSample ir;
  RadarSample radar;
  while (true) {
    bool haveIR = g_irQueue.pop(ir);
    bool haveRadar = g_radarQueue.pop(radar);
    if (haveIR || haveRadar) {
      g_fusion.update(haveIR ? &ir : nullptr, haveRadar ? &radar : nullptr);
      if (g_publishFusion) {
        telemetry_publish_fusion(g_fusion.getTracksView());
        const char* ev = g_fusion.getLastEventKind();
        if (ev) {
          telemetry_publish_event(ev, 1, 0.7f);
        }
      }
    }
    vTaskDelay(tick);
  }
}

void taskTelemetry(void* param) {
  const TickType_t tick = pdMS_TO_TICKS(100);
  while (true) {
    telemetry_loop();
    vTaskDelay(tick);
  }
}

void taskUI(void* param) {
  const TickType_t tick = pdMS_TO_TICKS(500);
  while (true) {
    ui_blink();
    vTaskDelay(tick);
  }
}

// UART bridge: forward LD600CH data -> USB CDC Serial
void taskUartBridge(void* param) {
  uint8_t buf[256];
  static char hexBuf[512];
  static size_t hexLen = 0;
  static char cliLine[256];
  static size_t cliLen = 0;
  const TickType_t tick = pdMS_TO_TICKS(2);
  auto handleCliLine = [] (const char* line) {
    // Simple CLI: tokens separated by spaces, key=value pairs for params
    String s = String(line);
    s.trim();
    if (s.length() == 0) return;
    // Lowercase a copy for command matching
    String l = s;
    l.toLowerCase();
    auto print_ok = [](){ Serial.println("OK"); };
    auto print_err = [](const char* msg){ Serial.printf("ERR %s\r\n", msg); };
    if (l == "help") {
      Serial.println("Commands:");
      Serial.println("  help");
      Serial.println("  wifi status");
      Serial.println("  wifi portal");
      Serial.println("  wifi portal stop");
      Serial.println("  wifi connect ssid=<ssid> pass=<pass> save=1");
      Serial.println("  mqtt status | mqtt on | mqtt off | mqtt connect");
      Serial.println("  mqtt set host=<h> port=<p> user=<u> pass=<p> topic=<t>");
      Serial.println("  sensors status");
      Serial.println("  mlx status");
      Serial.println("  system info");
      return;
    }
    if (l.startsWith("wifi status")) {
      Serial.printf("WiFi: %s\r\n", wifi_manager_status());
      if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("IP: %s RSSI: %d\r\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
      }
      return;
    }
    if (l.startsWith("wifi connect")) {
      // parse ssid= pass= save=
      String ssid, pass; int save = 1;
      int p = 0;
      while (p < s.length()) {
        int sp = s.indexOf(' ', p);
        if (sp < 0) sp = s.length();
        String tok = s.substring(p, sp);
        int eq = tok.indexOf('=');
        if (eq > 0) {
          String key = tok.substring(0, eq); key.toLowerCase();
          String val = tok.substring(eq + 1);
          if (key == "ssid") ssid = val;
          else if (key == "pass") pass = val;
          else if (key == "save") save = val.toInt();
        }
        p = sp + 1;
      }
      if (ssid.length() == 0) { print_err("missing ssid"); return; }
      bool ok = wifi_manager_connect(ssid.c_str(), pass.c_str(), save != 0);
      Serial.println(ok ? "OK wifi connect" : "ERR wifi connect");
      Serial.printf("WiFi ip: %s\r\n", WiFi.localIP().toString().c_str());
      return;
    }
    if (l.startsWith("wifi portal")) {
      if (l.indexOf("stop") >= 0) {
        wifi_manager_stop_portal();
        telemetry_begin_server(LEXARO_WS_PATH, LEXARO_HTTP_PORT);
        Serial.println("OK wifi portal stop");
      } else {
        telemetry_stop_server();
        bool ok = wifi_manager_start_portal(nullptr, nullptr, 300000); // 5 min
        Serial.println(ok ? "OK wifi portal start" : "ERR wifi portal");
      }
      return;
    }
    if (l.startsWith("mqtt status")) {
      Serial.printf("MQTT connected=%s\r\n", mqtt_is_connected() ? "yes" : "no");
      Preferences prefs; prefs.begin("lexaro", true);
      String host = prefs.getString("mqtt_host", "");
      uint16_t port = prefs.getUShort("mqtt_port", 1883);
      String base = prefs.getString("mqtt_base", "lexaro");
      Serial.printf("host=%s port=%u topic=%s\r\n", host.c_str(), port, base.c_str());
      return;
    }
    if (l.startsWith("mqtt on")) { mqtt_enable(true); print_ok(); return; }
    if (l.startsWith("mqtt off")) { mqtt_enable(false); print_ok(); return; }
    if (l.startsWith("mqtt connect")) { bool ok = mqtt_connect(); Serial.println(ok ? "OK" : "ERR mqtt connect"); return; }
    if (l.startsWith("mqtt set")) {
      MqttConfig cfg; cfg.port = 1883; cfg.baseTopic = "lexaro";
      int p = 0;
      while (p < s.length()) {
        int sp = s.indexOf(' ', p);
        if (sp < 0) sp = s.length();
        String tok = s.substring(p, sp);
        int eq = tok.indexOf('=');
        if (eq > 0) {
          String key = tok.substring(0, eq); key.toLowerCase();
          String val = tok.substring(eq + 1);
          if (key == "host") cfg.host = val;
          else if (key == "port") cfg.port = (uint16_t)val.toInt();
          else if (key == "user") cfg.user = val;
          else if (key == "pass") cfg.pass = val;
          else if (key == "topic") cfg.baseTopic = val;
        }
        p = sp + 1;
      }
      mqtt_set_config(cfg);
      print_ok();
      return;
    }
    if (l.startsWith("sensors status")) {
      int y, m, d, hh, mm, ss; float rtcT = NAN;
      if (ds3231_read_tm(g_i2cSensor, y, m, d, hh, mm, ss, rtcT)) {
        Serial.printf("DS3231 %04d-%02d-%02d %02d:%02d:%02d T=%.2fC\r\n", y, m, d, hh, mm, ss, rtcT);
      } else Serial.println("DS3231 read fail");
      float tC = NAN, hPct = NAN;
      if (hdc1080_read(g_i2cSensor, tC, hPct)) {
        Serial.printf("HDC1080 T=%.2fC RH=%.1f%%\r\n", tC, hPct);
      } else Serial.println("HDC1080 read fail");
      float lux = NAN; uint16_t als = 0;
      if (veml6030_read(g_i2cSensor, lux, als)) {
        Serial.printf("VEML6030 ALS_raw=%u Lux~=%.2f\r\n", als, lux);
      } else Serial.println("VEML6030 read fail");
      return;
    }
    if (l.startsWith("sensors verbose")) {
      if (l.indexOf("on") >= 0) { g_verboseSensors = true; Serial.println("OK verbose on"); return; }
      if (l.indexOf("off") >= 0) { g_verboseSensors = false; Serial.println("OK verbose off"); return; }
      Serial.printf("verbose=%s\r\n", g_verboseSensors ? "on" : "off");
      return;
    }
    if (l.startsWith("ws ")) {
      if (l.indexOf("restart") >= 0) {
        telemetry_stop_server();
        telemetry_begin_server(LEXARO_WS_PATH, LEXARO_HTTP_PORT);
        Serial.println("OK ws restart");
        return;
      }
      if (l.indexOf("stop") >= 0) {
        telemetry_stop_server();
        Serial.println("OK ws stop");
        return;
      }
      if (l.indexOf("start") >= 0) {
        telemetry_begin_server(LEXARO_WS_PATH, LEXARO_HTTP_PORT);
        Serial.println("OK ws start");
        return;
      }
      Serial.println("ERR ws usage: ws start|stop|restart");
      return;
    }
    if (l.startsWith("mlx status")) {
      Serial.printf("MLX90640 %s\r\n", g_irManager.ok() ? "OK" : "NOT READY");
      return;
    }
    if (l.startsWith("mlx verbose")) {
      if (l.indexOf("on") >= 0) { g_verboseMlx = true; Serial.println("OK mlx verbose on"); return; }
      if (l.indexOf("off") >= 0) { g_verboseMlx = false; Serial.println("OK mlx verbose off"); return; }
      Serial.printf("mlx_verbose=%s\r\n", g_verboseMlx ? "on" : "off");
      return;
    }
    if (l.startsWith("mlx probe")) {
      mlx_probe_debug();
      return;
    }
    if (l.startsWith("mlx read")) {
      // usage: mlx read reg=0x8000
      uint16_t reg = 0x8000;
      int p = 0;
      while (p < s.length()) {
        int sp = s.indexOf(' ', p);
        if (sp < 0) sp = s.length();
        String tok = s.substring(p, sp);
        int eq = tok.indexOf('=');
        if (eq > 0) {
          String key = tok.substring(0, eq); key.toLowerCase();
          String val = tok.substring(eq + 1);
          if (key == "reg") reg = (uint16_t)strtoul(val.c_str(), nullptr, 0);
        }
        p = sp + 1;
      }
      uint16_t out = 0;
      int rc = i2c_read16_debug(g_i2c, MLX90640_I2C_ADDR, reg, out);
      Serial.printf("MLX READ reg=0x%04X rc=%d val=0x%04X\r\n", reg, rc, out);
      return;
    }
    if (l.startsWith("mlx ping")) {
      g_i2c.beginTransmission(MLX90640_I2C_ADDR);
      int rc = g_i2c.endTransmission();
      Serial.printf("MLX PING endTx=%d (0=OK)\r\n", rc);
      return;
    }
    if (l.startsWith("mlx speed")) {
      if (l.indexOf("100k") >= 0) { g_i2c.setClock(100000); Serial.println("OK mlx speed 100k"); return; }
      if (l.indexOf("400k") >= 0) { g_i2c.setClock(400000); Serial.println("OK mlx speed 400k"); return; }
      if (l.indexOf("1m") >= 0) { g_i2c.setClock(1000000); Serial.println("OK mlx speed 1m"); return; }
      Serial.println("ERR mlx speed usage: mlx speed 100k|400k|1m");
      return;
    }
    if (l.startsWith("mlx power")) {
      // active-low: HIGH=off, LOW=on
      Serial.println("MLX power cycle...");
      digitalWrite(POWER_MLX_PIN, HIGH);
      delay(100);
      digitalWrite(POWER_MLX_PIN, LOW);
      delay(200);
      // Try re-init
      bool ok = g_irManager.begin(g_i2c);
      Serial.println(ok ? "OK mlx reinit" : "ERR mlx reinit");
      return;
    }
    if (l.startsWith("ota ")) {
      if (l.indexOf("status") >= 0) { Serial.println(ota_status()); return; }
      if (l.indexOf("auto on") >= 0) { ota_set_auto(true); Serial.println("OK ota auto on"); return; }
      if (l.indexOf("auto off") >= 0) { ota_set_auto(false); Serial.println("OK ota auto off"); return; }
      if (l.indexOf("verbose") >= 0) {
        if (l.indexOf("on") >= 0) { ota_set_verbose(true); Serial.println("OK ota verbose on"); return; }
        if (l.indexOf("off") >= 0) { ota_set_verbose(false); Serial.println("OK ota verbose off"); return; }
        Serial.printf("ota_verbose=%s\r\n", ota_get_verbose() ? "on" : "off"); return;
      }
      if (l.indexOf("set") >= 0) {
        // ota set url=<URL>
        String url;
        int p = 0;
        while (p < s.length()) {
          int sp = s.indexOf(' ', p);
          if (sp < 0) sp = s.length();
          String tok = s.substring(p, sp);
          int eq = tok.indexOf('=');
          if (eq > 0) {
            String key = tok.substring(0, eq); key.toLowerCase();
            String val = tok.substring(eq + 1);
            if (key == "url") url = val;
          }
          p = sp + 1;
        }
        if (url.length() == 0) { Serial.println("ERR ota missing url"); return; }
        ota_set_url(url);
        Serial.println("OK ota set");
        return;
      }
      if (l.indexOf("run") >= 0 || l.indexOf("start") >= 0) {
        String err;
        bool ok = ota_run_now(&err);
        if (!ok) Serial.printf("ERR ota %s\r\n", err.c_str());
        return;
      }
      Serial.println("ERR ota usage: ota set url=<URL> | ota run|start | ota status | ota auto on|off | ota verbose on|off|status");
      return;
    }
    if (l.startsWith("system info")) {
      Serial.printf("Uptime_ms=%lu FreeHeap=%u\r\n", millis(), (unsigned)ESP.getFreeHeap());
      return;
    }
    print_err("unknown");
  };
  for (;;) {
    // Read CLI from USB CDC Serial
    int ns = Serial.available();
    if (ns > 0) {
      if (ns > (int)sizeof(buf)) ns = sizeof(buf);
      int rs = Serial.readBytes(buf, ns);
      for (int i = 0; i < rs; ++i) {
        char c = (char)buf[i];
        if (c == '\r' || c == '\n') {
          if (cliLen > 0) {
            cliLine[cliLen] = 0;
            handleCliLine(cliLine);
            cliLen = 0;
          }
        } else {
          if (cliLen < sizeof(cliLine) - 1) cliLine[cliLen++] = c;
          else cliLen = 0;
        }
      }
    }
    // Si OTA en cours, alléger la tâche UART pour éviter overflow
    if (ota_is_running()) { vTaskDelay(pdMS_TO_TICKS(20)); continue; }
    int n = RadarUART.available();
    if (n > 0) {
      if (n > (int)sizeof(buf)) n = sizeof(buf);
      int r = RadarUART.readBytes(buf, n);
      if (r > 0) {
        if (g_verboseSensors) Serial.write(buf, r);
        // Accumuler uniquement les caractères hex et parser en flux
        for (int i = 0; i < r; ++i) {
          char c = (char)buf[i];
          bool isHex = (c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f');
          if (isHex) {
            if (hexLen < sizeof(hexBuf) - 1) {
              hexBuf[hexLen++] = (char)toupper((unsigned char)c);
              hexBuf[hexLen] = 0;
            } else {
              hexLen = 0; // overflow protection
            }
          }
        }
        // Parsing AA55 frames in streaming buffer
        auto hexToByte = [](char hi, char lo)->int {
          auto decode = [](char ch)->int {
            if (ch >= '0' && ch <= '9') return ch - '0';
            if (ch >= 'A' && ch <= 'F') return 10 + (ch - 'A');
            return -1;
          };
          int h = decode(hi), l = decode(lo);
          if (h < 0 || l < 0) return -1;
          return (h << 4) | l;
        };
        // We might have multiple frames; consume progressively
        size_t p = 0;
        while (hexLen >= 4 + 22) {
          // find 'AA55'
          while (p + 4 <= hexLen && !(hexBuf[p] == 'A' && hexBuf[p+1] == 'A' && hexBuf[p+2] == '5' && hexBuf[p+3] == '5')) {
            p++;
          }
          if (p + 4 > hexLen) break;
          if (p + 4 + 22 > hexLen) break; // wait more digits
          // decode 11 bytes after AA55
          uint8_t b[11];
          bool ok = true;
          size_t off = p + 4;
          for (int bi = 0; bi < 11; ++bi) {
            int v = hexToByte(hexBuf[off], hexBuf[off+1]);
            if (v < 0) { ok = false; break; }
            b[bi] = (uint8_t)v;
            off += 2;
          }
          if (ok) {
            uint8_t presence = b[0];
            uint8_t resp = b[1];
            uint8_t heart = b[2];
            uint8_t bucket = b[3];
            uint8_t motion = b[4];
            uint16_t distCm = (uint16_t)b[6] | ((uint16_t)b[7] << 8);
            g_decodedPresence = presence;
            g_decodedRespBpm = (float)resp;
            g_decodedHeartBpm = (float)heart;
            g_decodedBucket = bucket;
            g_decodedMotion = motion;
            g_decodedDistCm = distCm;
            g_decodedTsUs = micros64();
            if (g_verboseSensors) {
              Serial.printf("LD6002H pres=%u resp=%u heart=%u bucket=%u motion=%u dist_cm=%u\r\n",
                            presence, resp, heart, bucket, motion, (unsigned)distCm);
            }
          }
          // consume up to end of this frame (AA55 + 22 hex digits)
          size_t consume = p + 4 + 22;
          size_t remain = hexLen - consume;
          memmove(hexBuf, hexBuf + consume, remain);
          hexLen = remain;
          p = 0;
        }
      }
    }
    vTaskDelay(tick);
  }
}

// Periodic read of DS3231, VEML6030, HDC1080 and print to Serial
void taskSensors(void* param) {
  const TickType_t tick = pdMS_TO_TICKS(1000);
  for (;;) {
    int y, m, d, hh, mm, ss;
    float rtcTemp = NAN;
    bool okRtc = ds3231_read_tm(g_i2cSensor, y, m, d, hh, mm, ss, rtcTemp);
    float tC = NAN, hPct = NAN;
    bool okHdc = hdc1080_read(g_i2cSensor, tC, hPct);
    float lux = NAN; uint16_t rawAls = 0;
    bool okAls = veml6030_read(g_i2cSensor, lux, rawAls);
    if (g_verboseSensors) {
      if (okRtc) Serial.printf("DS3231 %04d-%02d-%02d %02d:%02d:%02d T=%.2fC\r\n", y, m, d, hh, mm, ss, rtcTemp);
      else Serial.println("DS3231 read fail");
      if (okHdc) Serial.printf("HDC1080 T=%.2fC RH=%.1f%%\r\n", tC, hPct);
      else Serial.println("HDC1080 read fail");
      if (okAls) Serial.printf("VEML6030 ALS_raw=%u Lux~=%.2f\r\n", rawAls, lux);
      else Serial.println("VEML6030 read fail");
    }
    vTaskDelay(tick);
  }
}

// Capture microphone I2S and report RMS level
void taskMic(void* param) {
  const TickType_t tick = pdMS_TO_TICKS(200);
  int32_t buf[256];
  static float hp_prev_x = 0.0f;
  static float hp_prev_y = 0.0f;
  const float hp_alpha = 0.995f; // high-pass pole
  for (;;) {
    size_t br = 0;
    float acc = 0.0f;
    int count = 0;
    if (i2s_read(I2S_NUM_0, (void*)buf, sizeof(buf), &br, pdMS_TO_TICKS(50)) == ESP_OK && br > 0) {
      int samples = br / sizeof(int32_t); // interleaved R,L
      for (int i = 0; i + 1 < samples; i += 2) {
        float l = (float)buf[i] / 2147483648.0f;
        float r = (float)buf[i + 1] / 2147483648.0f;
        float x = 0.5f * (l + r);
        // High-pass filter to remove DC
        float y = x - hp_prev_x + hp_alpha * hp_prev_y;
        hp_prev_x = x;
        hp_prev_y = y;
        acc += y * y;
        count++;
      }
      if (count > 0 && g_verboseSensors) {
        float rms = sqrtf(acc / (float)count);
        Serial.printf("MIC_RMS=%.5f\r\n", rms);
      }
    }
    vTaskDelay(tick);
  }
}


