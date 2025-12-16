/* ---------------------------------------------
  CALCULO DO DELAY
    Clock do CPU: F_CPU = 16.000.000 Hz
    Prescaler: 1:1024
    Clock do Timer1: 16.000.000 / 1024 = 15.625 Hz
    Valor de Comparação (OCR1A): 15.625 -> 1 segundo.
    Repaet: 60 vezes.

  PROJETO DE ESTACAO METEOROLOGICA
    Funcionalidades:
      - Leitura de Temperatura (Graus Celsius)
      - Leitura de Pressão (hPa)

    Hardware Connection (Arduino Uno / ATmega328p):
      - SDA: PC4 (A4)
      - SCL: PC5 (A5)
      - CSB: +5V
      - SDO: GND
-------------------------------------------- */

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "twi.h"
#include "bmp280.h"
#include "lcd.h"

// --- Limites de Alarme ---
#define TEMPERATURE_THRESHOLD 3500   // 35.00 Graus Celsius
#define PRESSURE_THRESHOLD    50600  // 506.00 hPa

// --- Variáveis Globais ---
volatile uint8_t read_sensor_flag = 0;
volatile uint8_t seconds_count = 0;

// --- Variáveis do Timer ---
const uint16_t T1_init = 0;
const uint16_t T1_comparator = 15625;
// Gerenciar intervalo de leitura do sensor
const uint8_t read_interval = 5;

// --- Caracteres customizados ---
const uint8_t custom_chars[6][8] = {
  // ec_00 (Índice 0)
  {0b00001, 0b00011, 0b00011, 0b01110, 0b11100, 0b11000, 0b01000, 0b01000},
  // ec_01 (Índice 1)
  {0b10001, 0b11111, 0b00000, 0b00000, 0b11111, 0b10001, 0b01000, 0b00100},
  // ec_02 (Índice 2)
  {0b10000, 0b11000, 0b11000, 0b01110, 0b00111, 0b00011, 0b00010, 0b00010},
  // ec_10 (Índice 3)
  {0b01000, 0b11000, 0b11100, 0b01110, 0b00011, 0b00011, 0b00001, 0b00000},
  // ec_11 (Índice 4)
  {0b01000, 0b10001, 0b11111, 0b00000, 0b00000, 0b11111, 0b10001, 0b00000},
  // ec_12 (Índice 5)
  {0b00010, 0b00011, 0b00111, 0b01110, 0b11000, 0b11000, 0b10000, 0b00000}
};

// --- Protótipos ---
void MCU_Init(void);
void TMR1_Init(void);

void LCD_UpdateData(int32_t temperature, uint32_t pressure);
void LCD_SendDecimal(int32_t value);
void LCD_LoadCustomChar(void);
void LCD_TelaInicial(void);
void LCD_PrintIcon(uint8_t column);

// ----------------------------------------
// TRATAMENTO DA INTERRUPÇÃO
// ----------------------------------------
ISR(TIMER1_COMPA_vect) {
  seconds_count++;

  if(seconds_count >= read_interval) {
    seconds_count = 0;
    read_sensor_flag = 1;
  }
}

// ----------------------------------------
// PROGRAMA PRINCIPAL
// ----------------------------------------
int main(void) {
  MCU_Init();

  LCD_Init();
  LCD_LoadCustomChar();

  BMP280_Init(); 
  BMP280_ReadCalibration();

  LCD_TelaInicial();

  int32_t temperature;
  uint32_t pressure;

  while (1) {
    if (read_sensor_flag) {
      read_sensor_flag = 0;

      BMP280_ReadSensor(&temperature, &pressure);

      // EXEMPLO DE APLICAÇÃO
      // LED PB0: Temperatura Alta
      if (temperature >= TEMPERATURE_THRESHOLD) PORTB |= (1<<PB0);
      else PORTB &= ~(1<<PB0);

      // LED PB1: Pressão Baixa
      if (pressure > PRESSURE_THRESHOLD) PORTB &= ~(1<<PB1);
      else PORTB |= (1<<PB1);

      LCD_UpdateData(temperature, pressure);
      LCD_PrintIcon(13);
    }
  }
  return 0;
}

// --------------------------------------------------------
// CONFIGURAÇÃO DE BAIXO NÍVEL
// --------------------------------------------------------
void MCU_Init(void) {
  // 1. Desabilita Watchdog
  wdt_disable();

  // 2. Desabilita Interrupções Globais
  cli();

  // 3. Configura Clock Prescaler para 1
  CLKPR = (1<<CLKPCE);  // Habilita a mudança
  CLKPR = 0x00;         // Define divisor como 1 (Clock total)

  // 4. Inicializa Portas
  DDRB  = 0xFF; 
  PORTB = 0x00;

  // 5. Configura protocolo I2C
  TWI_Init(); 
  
  // 6. Configura Timer
  TMR1_Init();

  // 7. Habilita Interrupção Global
  sei();

  _delay_ms(50);
}

void TMR1_Init(void) {
  // Zera os registradores de controle do Timer 1
  TCCR1A = 0;
  TCCR1B = 0;

  // Configura o Prescaler do Timer 1:1024 [101]
  TCCR1B |= (1<<CS12);
  TCCR1B &= ~(1<<CS11);
  TCCR1B |= (1<<CS10);

  // Limpa o timer ao comparar com OCR1A
  TCCR1B |= (1<<WGM12);

  // Inicializa Registradores do contador e o valor de comparação
  TCNT1 = T1_init;
  OCR1A = T1_comparator;

  // Habilita a Interrupção de Comparação A do Timer1
  TIMSK1 |= (1<<OCIE1A);
}

// --- Personalização de Telas LCD ---
void LCD_UpdateData(int32_t temperature, uint32_t pressure) {
  LCD_Clear();
  LCD_SetCursor(0, 0);
  LCD_SendString("T: ");
  LCD_SendDecimal(temperature);
  LCD_SendData(0xDF); // Caractere 'º'
  LCD_SendString("C");
  
  char buffer[16];
  ltoa(pressure/100, buffer, 10);
  LCD_SetCursor(1, 0);
  LCD_SendString("P: ");
  LCD_SendString(buffer);
  LCD_SendString(" hPa");
}

void LCD_SendDecimal(int32_t value) {
  char buffer[12];
  
  if (value < 0) {
    LCD_SendData('-');
    value = -value;
  }

  int32_t parte_inteira = value / 100;
  ltoa(parte_inteira, buffer, 10);
  LCD_SendString(buffer);
  LCD_SendData('.');

  int32_t parte_fracionaria = value % 100;

  if (parte_fracionaria < 10) {
    LCD_SendData('0');
  }

  ltoa(parte_fracionaria, buffer, 10);
  LCD_SendString(buffer);
}

void LCD_LoadCustomChar(void) {
  uint8_t i, j;
  // Aponta para o início da memória CGRAM
  LCD_SendCommand(0x40); 

  for (i = 0; i < 6; i++) {
    for (j = 0; j < 8; j++) {
      LCD_SendData(custom_chars[i][j]);
    }
  }
  // Retorna o cursor para a memória de display
  LCD_SendCommand(0x80); 
}

void LCD_TelaInicial(void) {
  LCD_SetCursor(0, 3);
  LCD_SendString("-- UFBA --");
  _delay_ms(2000);
  LCD_Clear();
  LCD_SetCursor(0, 4);
  LCD_SendString("Sistemas");
  LCD_SetCursor(1, 0);
  LCD_SendString("Microcontrolados");
}

void LCD_PrintIcon(uint8_t column) {
  LCD_SetCursor(0, column);
  LCD_SendData(0);
  LCD_SendData(1);
  LCD_SendData(2);
  LCD_SetCursor(1, column);
  LCD_SendData(3);
  LCD_SendData(4);
  LCD_SendData(5);
}