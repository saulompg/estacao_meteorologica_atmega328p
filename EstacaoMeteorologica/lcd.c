/* ------------------------------------------------------------
 File: lcd.c
 Author: Saulo
 Comments: Implementação da comunicação com LCD via protocolo I2C - PCF8574
 Comunicação em modo 4 bits - envio de nibble alto seguido do nibble baixo
------------------------------------------------------------ */

#include "lcd.h"

void LCD_SendNibble(uint8_t nibble) {
    uint8_t data = nibble | LCD_BACKLIGHT;

    TWI_Start();
    TWI_Write(LCD_ADDR << 1);   // Endereço do LCD + Write
    TWI_Write(data);            // Envia o nibble
    TWI_Write(data | LCD_EN);   // Habilita En
    _delay_us(1); // Pequeno delay para o LCD capturar o dado
    TWI_Write(data & ~LCD_EN);  // Desabilita En
    TWI_Stop();
}

void LCD_SendCommand(uint8_t cmd) {
    uint8_t highNibble = (cmd & 0xF0) & ~LCD_RS;
    uint8_t lowNibble = ((cmd << 4) & 0xF0) & ~LCD_RS;
    LCD_SendNibble(highNibble);
    LCD_SendNibble(lowNibble);
}

void LCD_SendData(uint8_t data) {
    uint8_t highNibble = (data & 0xF0) | LCD_RS;
    uint8_t lowNibble = ((data << 4) & 0xF0) | LCD_RS;
    LCD_SendNibble(highNibble);
    LCD_SendNibble(lowNibble);
}

void LCD_Init(void) {
    LCD_SendCommand(0x03);
    LCD_SendCommand(0x02);
    _delay_ms(2);
    LCD_SendCommand(0x28);
    LCD_SendCommand(0x0C);
    LCD_SendCommand(0x06);
    LCD_Clear();
}

void LCD_Clear(void) {
    LCD_SendCommand(0x01);
    _delay_us(50); // Delay para o comando de limpeza
}

void LCD_SetCursor(uint8_t row, uint8_t col) {
    uint8_t address = (row == 0) ? 0x80 + col : 0xC0 + col;
    LCD_SendCommand(address);
}

void LCD_SendString(const char *str) {
    while (*str) {
        LCD_SendData((uint8_t)(*str));
        str++;
    }
}