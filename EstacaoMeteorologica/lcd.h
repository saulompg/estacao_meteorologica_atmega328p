/* ------------------------------------------------------------
 File: lcd.h
 Author: Saulo
 Comments: Header para comunicação com LCD via protocolo I2C - PCF8574
 Comunicação em modo 4 bits - envio de nibble alto seguido do nibble baixo
------------------------------------------------------------ */

#ifndef LCD_H
#define	LCD_H

#include <stdint.h>
#include "twi.h"

// Definições de Endereçamento no PCF8574
#define LCD_ADDR         0x20 // Endereço I2C (7 bits)
#define LCD_RS           0x01 // Bit 0 (Register Select)
#define LCD_RW           0x02 // Bit 1 (Read/Write)
#define LCD_EN           0x04 // Bit 2 (Enable)
#define LCD_BACKLIGHT    0x08 // Bit 3 (Backlight Control)
// Bits 4-7 são usados para os dados (D4-D7)

void LCD_SendNibble(uint8_t nibble);
void LCD_SendCommand(uint8_t cmd);
void LCD_SendData(uint8_t data);
void LCD_Init(void);	
void LCD_Clear(void);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_SendString(const char *str);

#endif	/* LCD_H */