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
 -------------------------------------------- */

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "twi.h"
#include "bmp280.h"

// --- Limites de Alarme ---
#define TEMP_THRESHOLD   3500   // 35.00 Graus Celsius
#define PRESS_THRESHOLD  50600  // 506.00 hPa

// --- Variáveis Globais ---
volatile uint8_t read_sensor_flag = 0;
volatile uint8_t seconds_count = 0;

// --- Variáveis do Timer ---
const uint16_t T1_init = 0;
const uint16_t T1_comp = 15625;
// Gerenciar intervalo de leitura do sensor
const uint8_t read_interval = 5;

// --- Protótipos ---
void MCU_Init(void);
void TMR1_Init(void);

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

  BMP280_Init(); 
  BMP280_ReadCalibration();

  int32_t temp;
  uint32_t press;
    
  while (1) {
    if (read_sensor_flag) {
      read_sensor_flag = 0;

      BMP280_ReadSensor(&temp, &press);

      // EXEMPLO DE APLICAÇÃO
      // LED PB0: Temperatura Alta
      if (temp >= TEMP_THRESHOLD) PORTB |= (1<<PB0);
      else PORTB &= ~(1<<PB0);

      // LED PB1: Pressão Baixa
      if (press > PRESS_THRESHOLD) PORTB &= ~(1<<PB1);
      else PORTB |= (1<<PB1);
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
  OCR1A = T1_comp;

  // Habilita a Interrupção de Comparação A do Timer1
  TIMSK1 |= (1<<OCIE1A);
}