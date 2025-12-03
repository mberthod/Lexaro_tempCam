// TMP117 temperature sensor (I2C)
#pragma once

#include <Arduino.h>
#include <Wire.h>

class TMP117Driver {
public:
  TMP117Driver() : _addr(0x48), _wire(nullptr), _ok(false) {}
  bool begin(TwoWire& wire, uint8_t addr = 0x48);
  bool readTemperatureC(float& outC);
  bool ok() const { return _ok; }
private:
  uint8_t _addr;
  TwoWire* _wire;
  bool _ok;
  bool readRegister(uint8_t reg, uint16_t& val);
};


