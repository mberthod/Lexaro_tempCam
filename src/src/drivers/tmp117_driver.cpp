// TMP117 driver implementation
#include "tmp117_driver.h"

// TMP117 registers
static constexpr uint8_t REG_TEMP = 0x00;
static constexpr uint8_t REG_CONFIG = 0x01;
static constexpr uint16_t CONFIG_INIT = 0x0220; // 4Hz, continuous

bool TMP117Driver::begin(TwoWire& wire, uint8_t addr) {
  _wire = &wire;
  _addr = addr;
  // Write a simple configuration
  _wire->beginTransmission(_addr);
  _wire->write(REG_CONFIG);
  _wire->write((CONFIG_INIT >> 8) & 0xFF);
  _wire->write((CONFIG_INIT >> 0) & 0xFF);
  if (_wire->endTransmission() != 0) {
    _ok = false;
    return false;
  }
  _ok = true;
  return true;
}

bool TMP117Driver::readRegister(uint8_t reg, uint16_t& val) {
  if (!_wire) return false;
  _wire->beginTransmission(_addr);
  _wire->write(reg);
  if (_wire->endTransmission(false) != 0) return false;
  if (_wire->requestFrom((int)_addr, 2) != 2) return false;
  uint8_t msb = _wire->read();
  uint8_t lsb = _wire->read();
  val = ((uint16_t)msb << 8) | lsb;
  return true;
}

bool TMP117Driver::readTemperatureC(float& outC) {
  uint16_t raw = 0;
  if (!readRegister(REG_TEMP, raw)) {
    return false;
  }
  // TMP117: 7.8125e-3 °C per LSB, two's complement
  int16_t sraw = (int16_t)raw;
  outC = (float)sraw * 0.0078125f;
  return true;
}


