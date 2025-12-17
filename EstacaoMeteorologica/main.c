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
#include <stdlib.h>

#include "twi.h"
#include "bmp280.h"
#include "dht11.h"
#include "lcd.h"

// --- Definições de Hardware ---
#define BUTTON_PIN    PD2
#define BUTTON_PORT   PORTD
#define BUTTON_PIN_IN PIND

// --- Limites de Alarme ---
#define TEMPERATURE_THRESHOLD 3500   // 35.00 Graus Celsius
#define PRESSURE_THRESHOLD    80000  // 800.00 hPa
#define HUMIDITY_THRESHOLD    30     // 30% de humidade

// --- Variáveis Globais ---
volatile uint8_t read_sensor_flag = 0;
volatile uint8_t seconds_count = 0;

// --- Estado da tela: 0 (Temp/Pres), 1 (Pres/Hum), 2 (Hum/Temp) ---
uint8_t current_screen = 0;

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
void Check_Button(void);

void LCD_RefreshScreen(int32_t temperature, uint32_t pressure, DHT11_Data dht11);
void LCD_FormatTemperature(int32_t temperature);
void LCD_FormatPressure(uint32_t pressure);
void LCD_FormatHumidity(DHT11_Data dht11);

void LCD_LoadCustomChar(void);
void LCD_HomeScreen(void);
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

  LCD_HomeScreen();

  int32_t temperature = 0;
  uint32_t pressure = 0;
  DHT11_Data dht11 = {0,0,0,0,1};

  // força a primeira leitura
  BMP280_ReadSensor(&temperature, &pressure);
  dht11 = DHT11_Read();
  LCD_RefreshScreen(temperature, pressure, dht11);

  while (1) {
    // polling contínuo
    if (!(BUTTON_PIN_IN & (1 << BUTTON_PIN))) { // checa se LOW (Botão pressionado)
      _delay_ms(50);
      if (!(BUTTON_PIN_IN & (1 << BUTTON_PIN))) {
        current_screen++;
        if (current_screen > 2) current_screen = 0;
        LCD_RefreshScreen(temperature, pressure, dht11);
        while(!(BUTTON_PIN_IN & (1 << BUTTON_PIN))); // espera o botão ser liberado
      }
    }

    // leitura periódica dos sensores
    if (read_sensor_flag) {
      read_sensor_flag = 0;

      BMP280_ReadSensor(&temperature, &pressure);
      dht11 = DHT11_Read();

      // ALARMES
      // LED PB0: Temperatura Alta
      if (temperature >= TEMPERATURE_THRESHOLD) PORTB |= (1<<PB0);
      else PORTB &= ~(1<<PB0);

      // LED PB1: Pressão Baixa
      if (pressure > PRESSURE_THRESHOLD) PORTB &= ~(1<<PB1);
      else PORTB |= (1<<PB1);

      // LED PB2: Humidade Baixa
      if (dht11.humidity_int < HUMIDITY_THRESHOLD) PORTB |= (1<<PB2);
      else PORTB &= ~(1<<PB2); 

      LCD_RefreshScreen(temperature, pressure, dht11);
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

  // 4. Configura GPIO
  DDRB  = 0xFF; 
  PORTB = 0x00;

  DDRD &= ~(1 << BUTTON_PIN);
  PORTD |= (1 << BUTTON_PIN);

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
  TCCR1B |= (1<<CS12) | (1<<CS10);

  // Limpa o timer ao comparar com OCR1A - Modo CTC
  TCCR1B |= (1<<WGM12);

  // Inicializa Registradores do contador e o valor de comparação
  TCNT1 = T1_init;
  OCR1A = T1_comparator;

  // Habilita a Interrupção de Comparação A do Timer1
  TIMSK1 |= (1<<OCIE1A);
}

// --- Personalização de Telas LCD ---
void LCD_RefreshScreen(int32_t temperature, uint32_t pressure, DHT11_Data humidity) {
  LCD_Clear();

  switch (current_screen) {
    case 0: // TELA 0: Temp (L1) e Pressão (L2)
      LCD_SetCursor(0, 0);
      LCD_SendString("T: "); LCD_FormatTemperature(temperature);
      LCD_SetCursor(1, 0);
      LCD_SendString("P: "); LCD_FormatPressure(pressure);
      break;

    case 1: // TELA 1: Pressão (L1) e Umidade (L2)
      LCD_SetCursor(0, 0);
      LCD_SendString("P: "); LCD_FormatPressure(pressure);
      LCD_SetCursor(1, 0);
      LCD_SendString("H: "); LCD_FormatHumidity(humidity);
      break;

    case 2: // TELA 2: Umidade (L1) e Temp (L2)
      LCD_SetCursor(0, 0);
      LCD_SendString("H: "); LCD_FormatHumidity(humidity);
      LCD_SetCursor(1, 0);
      LCD_SendString("T: "); LCD_FormatTemperature(temperature);
      break;
  }

  LCD_PrintIcon(12);
}

void LCD_FormatTemperature(int32_t temperature) {
  char buffer[8];

  if (temperature < 0) {
    LCD_SendData('-');
    temperature = -temperature;
  }
  // envia parte inteira
  ltoa(temperature/100, buffer, 10);
  LCD_SendString(buffer);
  LCD_SendData('.');
  // envia parte decimal
  ltoa(temperature%100, buffer, 10);
  if((temperature%100) < 10) LCD_SendString("0"); 
  LCD_SendString(buffer);

  LCD_SendData(0xDF); LCD_SendString("C");
}

void LCD_FormatPressure(uint32_t pressure) {
  char buffer[6];
  ltoa(pressure/100, buffer, 10); // Exibindo hPa inteiro
  LCD_SendString(buffer);
  LCD_SendString(" hPa");
}

void LCD_FormatHumidity(DHT11_Data dht11) {
  char buffer[4];
  if (dht11.error) {
    LCD_SendString("-- %");
  } else {
    itoa(dht11.humidity_int, buffer, 10);
    LCD_SendString(buffer);
    LCD_SendData('.');
    itoa(dht11.humidity_dec, buffer, 10);
    LCD_SendString(buffer);
    LCD_SendString(" %");
  }
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

void LCD_HomeScreen(void) {
  LCD_SetCursor(0, 3);
  LCD_SendString("-- UFBA --");
  _delay_ms(1400);
  LCD_Clear();
  LCD_SetCursor(0, 4);
  LCD_SendString("Sistemas");
  LCD_SetCursor(1, 0);
  LCD_SendString("Microcontrolados");
  _delay_ms(1400);
  LCD_Clear();
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