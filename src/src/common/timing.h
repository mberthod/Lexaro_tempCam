// timing.h — time utilities
#pragma once

#include <stdint.h>
extern "C" {
  #include "esp_timer.h"
}

static inline uint64_t micros64() {
  return (uint64_t)esp_timer_get_time(); // microseconds
}

static inline uint64_t millis64() {
  return micros64() / 1000ULL;
}


