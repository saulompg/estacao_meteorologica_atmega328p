/* ------------------------------------------------------------
 File: bmp280.h
 Author: Saulo
 Comments: Header para comunicação com BMP280 via protocolo I2C
------------------------------------------------------------ */

#ifndef BMP280_H
#define	BMP280_H

#include <stdint.h>
#include "twi.h"

// Definições de Endereço
#define BMP280_ADDR      0x76 // Endereço I2C (7 bits)
#define BMP280_REG_CTRL  0xF4 // Controle de medição
#define BMP280_REG_DATA  0xF7 // Início dos dados (Pressão MSB)
#define BMP280_REG_CALIB 0x88 // Início dos dados de calibração

void BMP280_Init(void);	
void BMP280_ReadCalibration(void);
void BMP280_ReadSensor(int32_t *temp_final, int32_t *press_final);

#endif	/* BMP280_H */

