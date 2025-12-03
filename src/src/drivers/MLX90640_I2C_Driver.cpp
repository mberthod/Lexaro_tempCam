#include <Arduino.h>
#include <Wire.h>

extern TwoWire g_i2c; // défini dans LexaroTempCam.cpp

extern "C" {

int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nWordsRead, uint16_t *data)
{
  // Lecture par tranches pour éviter de dépasser le buffer I2C (ESP32 ~128 octets)
  const uint16_t maxWordsPerChunk = 16; // 16 mots = 32 octets
  uint16_t addr = startAddress;
  uint16_t remaining = nWordsRead;
  uint16_t offset = 0;

  while (remaining > 0) {
    uint16_t chunk = (remaining > maxWordsPerChunk) ? maxWordsPerChunk : remaining;

    g_i2c.beginTransmission(slaveAddr);
    g_i2c.write((uint8_t)((addr >> 8) & 0xFF));
    g_i2c.write((uint8_t)(addr & 0xFF));
    int rc = g_i2c.endTransmission(false); // repeated-start
    if (rc != 0) {
      return rc; // valeur négative/positive suivant l'implémentation Wire
    }

    int bytesToRead = (int)chunk * 2;
    int got = g_i2c.requestFrom((int)slaveAddr, bytesToRead, (int)true);
    if (got != bytesToRead) {
      return -1; // lecture incomplète
    }

    for (uint16_t i = 0; i < chunk; ++i) {
      uint8_t ms = g_i2c.read();
      uint8_t ls = g_i2c.read();
      data[offset + i] = ((uint16_t)ms << 8) | ls;
    }

    addr += chunk;
    offset += chunk;
    remaining -= chunk;
  }

  return 0;
}

int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data)
{
  g_i2c.beginTransmission(slaveAddr);
  g_i2c.write((uint8_t)((writeAddress >> 8) & 0xFF));
  g_i2c.write((uint8_t)(writeAddress & 0xFF));
  g_i2c.write((uint8_t)((data >> 8) & 0xFF));
  g_i2c.write((uint8_t)(data & 0xFF));
  int rc = g_i2c.endTransmission(true);
  return rc;
}

void MLX90640_I2CFreqSet(int freq)
{
  g_i2c.setClock((uint32_t)freq);
}

int MLX90640_I2CGeneralReset(void)
{
  // General Call reset: adresse 0x00, octet 0x06
  g_i2c.beginTransmission(0x00);
  g_i2c.write((uint8_t)0x06);
  int rc = g_i2c.endTransmission(true);
  delay(5);
  return (rc == 0) ? 0 : -1;
}

} // extern "C"


