#include <Arduino.h>
#include <Wire.h>

extern TwoWire g_i2c; // défini dans LexaroTempCam.cpp

extern "C" {

int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nWordsRead, uint16_t *data) {
  // Lire par tranches pour respecter la taille de buffer I2C (ESP32 ~128B)
  // 16 mots = 32 octets
  const int maxWordsPerChunk = 16;
  uint16_t addr = startAddress;
  int remaining = (int)nWordsRead;
  int offset = 0;
  while (remaining > 0) {
    int chunkWords = remaining > maxWordsPerChunk ? maxWordsPerChunk : remaining;
    // Positionner l'adresse de départ
    g_i2c.beginTransmission(slaveAddr);
    g_i2c.write((uint8_t)((addr >> 8) & 0xFF));
    g_i2c.write((uint8_t)(addr & 0xFF));
    int rc = g_i2c.endTransmission(false); // repeated start
    if (rc != 0) return rc;
    int bytesToRead = chunkWords * 2;
    int got = g_i2c.requestFrom((int)slaveAddr, bytesToRead, (int)true);
    if (got != bytesToRead) return -1;
    for (int i = 0; i < chunkWords; i++) {
      uint8_t msb = g_i2c.read();
      uint8_t lsb = g_i2c.read();
      data[offset + i] = ((uint16_t)msb << 8) | lsb;
    }
    addr += (uint16_t)chunkWords;
    offset += chunkWords;
    remaining -= chunkWords;
  }
  return 0;
}

int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data) {
  g_i2c.beginTransmission(slaveAddr);
  g_i2c.write((uint8_t)((writeAddress >> 8) & 0xFF));
  g_i2c.write((uint8_t)(writeAddress & 0xFF));
  g_i2c.write((uint8_t)((data >> 8) & 0xFF));
  g_i2c.write((uint8_t)(data & 0xFF));
  int rc = g_i2c.endTransmission(true);
  return rc;
}

void MLX90640_I2CFreqSet(int freq) {
  g_i2c.setClock((uint32_t)freq);
}

} // extern "C"


