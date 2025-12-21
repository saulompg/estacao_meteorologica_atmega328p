/* ------------------------------------------------------------
 File: lcd.c
 Author: Saulo
 Comments: Implementação da comunicação com LCD via protocolo I2C - PCF8574
 Comunicação em modo 4 bits - envio de nibble alto seguido do nibble baixo
------------------------------------------------------------ */

#include "lcd.h"

void LCD_PulseEnable(uint8_t data) {
    TWI_Write(data | LCD_EN);
    _delay_us(1);
    TWI_Write(data & ~LCD_EN);
    _delay_us(50);
}

void LCD_SendNibble(uint8_t nibble) {
    uint8_t data = nibble | LCD_BACKLIGHT;

    TWI_Start();
    TWI_Write(LCD_ADDR << 1);   // Endereço do LCD + Write
    TWI_Write(data);            // Prepara os dados no pino
    LCD_PulseEnable(data);      // Pulsa o Enable
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
    _delay_ms(50);
    // Sequência de Reset (0x03 repetido 3 vezes)
    LCD_SendNibble(0x30 & ~LCD_RS);
    _delay_ms(5);

    LCD_SendNibble(0x30 & ~LCD_RS);
    _delay_ms(150);

    LCD_SendNibble(0x30 & ~LCD_RS);

    // Mudar para modo 4-bits
    LCD_SendNibble(0x20 & ~LCD_RS); 
    _delay_ms(1);

    // Configuração inicial
    LCD_SendCommand(0x28);
    LCD_SendCommand(0x0C);
    LCD_SendCommand(0x06);

    LCD_Clear();
}

void LCD_Clear(void) {
    LCD_SendCommand(0x01);
    _delay_ms(2); // Delay para o comando de limpeza
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