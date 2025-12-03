#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file MLX90640_I2C_Driver.h
 * @brief Couche d’accès I2C minimale pour la librairie Melexis MLX90640 (Arduino Wire).
 *
 * Les lectures sont segmentées en blocs pour éviter de dépasser le buffer I2C.
 */

int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nWordsRead, uint16_t *data);
int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data);
void MLX90640_I2CFreqSet(int freq);
int MLX90640_I2CGeneralReset(void);

#ifdef __cplusplus
}
#endif