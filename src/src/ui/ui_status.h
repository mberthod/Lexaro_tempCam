// ui_status.h — simple UI status handling
#pragma once

#include <Arduino.h>
#include "../config/Config.h"

enum class UIStatus {
  Booting,
  Ready
};

inline void ui_init() {
  pinMode(LEXARO_LED_PIN, OUTPUT);
  digitalWrite(LEXARO_LED_PIN, LOW);
}

inline void ui_set_status(UIStatus st) {
  if (st == UIStatus::Booting) {
    digitalWrite(LEXARO_LED_PIN, LOW);
  } else {
    digitalWrite(LEXARO_LED_PIN, HIGH);
  }
}

inline void ui_blink() {
  static bool on = false;
  on = !on;
  digitalWrite(LEXARO_LED_PIN, on ? HIGH : LOW);
}


