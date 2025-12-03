// Config.h — Lexaro configuration (pins, timings, network, tasks)
#pragma once

#include <stdint.h>

// -------- Hardware pins (ESP32-S3 default, adjustable) --------


#define POWER_CAM_PIN 8
#define POWER_SD_PIN 33
#define POWER_MIC_PIN 34
#define POWER_VEML_PIN 35
#define POWER_SHT_PIN 36
#define POWER_MLX_PIN 37
#define POWER_RADAR_PIN 17
#define LEXARO_I2C_SDA 9
#define LEXARO_I2C_SCL 10
#define LEXARO_LED_PIN 21
#define LEXAROSENSOR_I2C_SDA 48
#define LEXAROSENSOR_I2C_SCL 47

// Microphone I2S pins
#define I2S_MIC_WS 11
#define I2S_MIC_SCK 12
#define I2S_MIC_SD 13

// I2C device addresses
#define DS3231_ADDR 0x68
#define HDC1080_ADDR 0x40
// VEML6030 may respond on 0x10 or 0x48 depending on ADD pin; try in this order
#define VEML6030_ADDR_A 0x10
#define VEML6030_ADDR_B 0x48



// -------- Sensors addresses/dimensions --------
#define TMP117_DEFAULT_ADDR 0x48
#define MLX90640_I2C_ADDR 0x33

#define IR_WIDTH 32
#define IR_HEIGHT 24

// -------- Network / WebSocket (native) --------
#define LEXARO_HTTP_PORT 80
#define LEXARO_WS_PATH "/ws"

#define LEXARO_AP_SSID "Lexaro-TempCam"
#define LEXARO_AP_PASS "lexaro123"

// -------- Task stack sizes and priorities --------
#define LEXARO_STACK_IR    8192
#define LEXARO_STACK_RADAR 8192
#define LEXARO_STACK_FUSION 8192
#define LEXARO_STACK_TELEM 8192
#define LEXARO_STACK_UI    8192

// Augmenter la pile de la tâche IR pour le driver MLX (EEPROM + frame buffers)
#undef LEXARO_STACK_IR
#define LEXARO_STACK_IR    12288

#define LEXARO_PRIO_IR     5
#define LEXARO_PRIO_RADAR  6
#define LEXARO_PRIO_FUSION 4
#define LEXARO_PRIO_TELEM  3
#define LEXARO_PRIO_UI     2


