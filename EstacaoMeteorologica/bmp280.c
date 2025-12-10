/* ------------------------------------------------------------
 File: bmp280.c
 Author: Saulo
 Comments: Implementacao da comunicação com BMP280 via protocolo I2C
------------------------------------------------------------ */

#include "bmp280.h"

// --- Variáveis Auxiliares ---
static int32_t t_fine;

struct BMP280_Calib {
  uint16_t dig_T1; int16_t  dig_T2; int16_t  dig_T3;
  uint16_t dig_P1; int16_t  dig_P2; int16_t  dig_P3;
  int16_t  dig_P4; int16_t  dig_P5; int16_t  dig_P6;
  int16_t  dig_P7; int16_t  dig_P8; int16_t  dig_P9;
} calib;

// --- Funções Auxiliares ---
void BMP280_WriteReg(uint8_t reg, uint8_t value) {
  TWI_Start();
  TWI_Write(BMP280_ADDR << 1); // Endereço + Write (0)
  TWI_Write(reg);              // Ponteiro do registrador
  TWI_Write(value);            // Valor a escrever
  TWI_Stop();
}

// --- Funções de Compensação ---
int32_t compensate_T(int32_t adc_T) {
  int32_t var1, var2, T;
  
  var1 = ((((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1))) * ((int32_t)calib.dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)calib.dig_T1))) >> 12) * ((int32_t)calib.dig_T3)) >> 14;
  
  t_fine = (int32_t)var1 + var2; // Atualiza variável global para uso na pressão
  
  T = (t_fine * 5 + 128) >> 8;
  return T; 
}

uint32_t compensate_P(int32_t adc_P) {
  int32_t var1, var2;
  uint32_t p;
  
  // O cálculo da pressão depende do t_fine (temperatura atual)
  var1 = (((int32_t)t_fine)>>1) - (int32_t)64000;
  var2 = (((var1>>2) * (var1>>2) >> 11) * ((int32_t)calib.dig_P6));
  var2 = var2 + ((var1 * ((int32_t)calib.dig_P5)) << 1);
  var2 = (var2>>2) + (((int32_t)calib.dig_P4) << 16);
  var1 = (((calib.dig_P3 * (((var1>>2) * (var1>>2)) >> 13)) >> 3) + ((((int32_t)calib.dig_P2) * var1) >> 1)) >> 18;
  var1 = ((((32768 + var1)) * ((int32_t)calib.dig_P1)) >> 15);
  
  if (var1 == 0) {
    return 0; // Evita divisão por zero
  }
  
  p = (((uint32_t) (((int32_t)1048576) - adc_P) - (var2>>12))) * 3125;
  if (p < 0x80000000) {
    p = (p << 1) / ((uint32_t) var1);
  } else {
    p = (p / (uint32_t)var1) * 2;
  }
  var1 = (((int32_t)calib.dig_P9) * ((int32_t) (((p>>3) * (p>>3))>>13)))>>12;
  var2 = (((int32_t) (p>>2)) * ((int32_t)calib.dig_P8))>>13;
  
  p = (uint32_t) ((int32_t)p + ((var1 + var2 + calib.dig_P7) >> 4));
  
  return p;
}

// --- Implementação das Funções Públicas ---
void BMP280_Init(void) {
  // Inicialização do Sensor (Normal mode, Oversampling x1)
  // Osrs_t x1 (001), Osrs_p x1 (001), Mode Normal (11)
  // Binário: 001 001 11 = 0x27
  BMP280_WriteReg(BMP280_REG_CTRL, 0x27);
}

void BMP280_ReadCalibration(void) {
  uint8_t buf[24];

  TWI_Start();
  TWI_Write(BMP280_ADDR << 1);
  TWI_Write(BMP280_REG_CALIB);

  TWI_Start();
  TWI_Write((BMP280_ADDR << 1) | 1);

  // Lê os primeiros 23 bytes com ACK
  for (int i = 0; i < 23; i++) {
    buf[i] = TWI_Read_ACK();
  }
  // Lê o último byte e responde com NACK
  buf[23] = TWI_Read_NACK();
  TWI_Stop();

  // Reconstrói a estrutura (Little Endian)
  calib.dig_T1 = (buf[1] << 8) | buf[0];    // 0x88
  calib.dig_T2 = (buf[3] << 8) | buf[2];    // 0x8A
  calib.dig_T3 = (buf[5] << 8) | buf[4];    // 0x8C
  calib.dig_P1 = (buf[7] << 8) | buf[6];    // 0x8E
  calib.dig_P2 = (buf[9] << 8) | buf[8];    // 0x90
  calib.dig_P3 = (buf[11] << 8) | buf[10];  // 0x92
  calib.dig_P4 = (buf[13] << 8) | buf[12];  // 0x94
  calib.dig_P5 = (buf[15] << 8) | buf[14];  // 0x96
  calib.dig_P6 = (buf[17] << 8) | buf[16];  // 0x98
  calib.dig_P7 = (buf[19] << 8) | buf[18];  // 0x9A
  calib.dig_P8 = (buf[21] << 8) | buf[20];  // 0x9C
  calib.dig_P9 = (buf[23] << 8) | buf[22];  // 0x9E
}

void BMP280_ReadSensor(int32_t *temp_final, uint32_t *press_final) {
  TWI_Start();
  TWI_Write(BMP280_ADDR << 1);
  TWI_Write(BMP280_REG_DATA);   // Ponteiro no início dos dados

  TWI_Start(); // Restart
  TWI_Write((BMP280_ADDR << 1) | 1); // Modo Leitura

  // Lê 6 bytes em sequência
  uint8_t p_msb = TWI_Read_ACK();
  uint8_t p_lsb = TWI_Read_ACK();	
  uint8_t p_xlsb = TWI_Read_ACK();
  uint8_t t_msb = TWI_Read_ACK();
  uint8_t t_lsb = TWI_Read_ACK();
  uint8_t t_xlsb = TWI_Read_NACK();   // Último byte = NACK
  TWI_Stop();

  // Montagem dos inteiros de 20 bits
  int32_t raw_press = ((int32_t)p_msb << 12) | ((int32_t)p_lsb << 4) | (p_xlsb >> 4);
  int32_t raw_temp  = ((int32_t)t_msb << 12) | ((int32_t)t_lsb << 4) | (t_xlsb >> 4);

  *temp_final = compensate_T(raw_temp); 
  *press_final = compensate_P(raw_press);
}
