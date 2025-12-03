// ring_buffer.h — simple thread-safe ring buffer for FreeRTOS
#pragma once

#include <string.h>
#include <Arduino.h>

template<typename T, size_t Capacity>
class RingBuffer {
public:
  RingBuffer() : _head(0), _tail(0), _count(0) {
    _mtx = xSemaphoreCreateMutex();
  }
  ~RingBuffer() {
    if (_mtx) {
      vSemaphoreDelete(_mtx);
    }
  }

  bool push(const T& item) {
    if (!_mtx) return false;
    if (xSemaphoreTake(_mtx, portMAX_DELAY) != pdTRUE) return false;
    bool ok = true;
    if (_count == Capacity) {
      // overwrite oldest
      _data[_head] = item;
      _head = (_head + 1) % Capacity;
      _tail = _head;
    } else {
      _data[_head] = item;
      _head = (_head + 1) % Capacity;
      _count++;
    }
    xSemaphoreGive(_mtx);
    return ok;
  }

  bool pop(T& out) {
    if (!_mtx) return false;
    if (xSemaphoreTake(_mtx, portMAX_DELAY) != pdTRUE) return false;
    bool ok = false;
    if (_count > 0) {
      out = _data[_tail];
      _tail = (_tail + 1) % Capacity;
      _count--;
      ok = true;
    }
    xSemaphoreGive(_mtx);
    return ok;
  }

  size_t size() const {
    return _count;
  }

private:
  T _data[Capacity];
  size_t _head;
  size_t _tail;
  size_t _count;
  SemaphoreHandle_t _mtx;
};


