/* ------------------------------------------------------------
 File: dht11.h
 Author: Fábio Santos
 Comments: Implementação da comunicação com DHT11
------------------------------------------------------------ */

#include "dht11.h"
#include <avr/interrupt.h>

// Emite o pulso de start para o DHT11
static void DHT11_Start(void) {
  DHT_PORT |= (1 << DHT_BIT);   // garante a linha em HIGH
  DHT_DDR  |= ( 1 << DHT_BIT);  // configura como saída 
  _delay_ms(100);               // pequeno tempo de estabilização
  
  DHT_PORT &= ~(1 << DHT_BIT);  // Envia o Sinal de Start (MCU puxa para LOW)
  _delay_ms(20);                // delay mínimo de 18 ms

  DHT_PORT |= (1<< DHT_BIT);    // Libera o Barramento (MCU puxa para HIGH)
  _delay_us(30);                // delay 20-40us
  
  DHT_DDR &= ~(1 << DHT_BIT);   // Configura como Entrada (Ouve o Sensor)
  DHT_PORT |= (1 << DHT_BIT);   // ativa pull-up interno
}

// Handshake do DHT
static uint8_t DHT11_CheckResponse(void) {
  // Espera o sensor baixar a tensao - 80us
  uint16_t timeout = DHT11_MAX_TIMEOUT;
  while (DHT_PIN & (1 << DHT_BIT)) {
    _delay_us(1);
    if (--timeout == 0) return 0; // Erro: Sensor não respondeu (ficou HIGH)
  }

  // Espera o sensor subir a tensao - 80us 
  timeout = DHT11_MAX_TIMEOUT;
  while (!(DHT_PIN & (1 << DHT_BIT))) {
    _delay_us(1);
    if (--timeout == 0) return 0; // Erro: Sensor travou em LOW
  }

  // Espera o sensor baixar novamente (inicio dos dados)
  timeout = DHT11_MAX_TIMEOUT;
  while (DHT_PIN & (1 << DHT_BIT)) {
    _delay_us(1);
    if (--timeout == 0) return 0; // Erro: Sensor não iniciou transmissão
  }

  return 1; 
}

static uint8_t DHT11_ReadByte(void) {
  uint8_t result = 0;

  for(uint8_t i = 0; i < 8; i++) {
    uint8_t timeout = DHT11_MAX_TIMEOUT;

    // Espera o fim do preambulo - 50us de LOW
    while(!(DHT_PIN & (1 << DHT_BIT))) {
      if (--timeout == 0) return 0;
    } 
    // Espera para medir sinal de HIGH - faixa de corte (~28us ou ~70us)
    _delay_us(40); 

    // Verificar se esta HIGH apos 40us
    if (DHT_PIN & (1 << DHT_BIT)) {
      result |= (1 << (7-i)); 

      // Espera o bit 1 terminar
      timeout = DHT11_MAX_TIMEOUT;
      while (DHT_PIN & (1 << DHT_BIT)) {
        if (--timeout == 0) return 0;
      }
    }
  }
  return result;
}

DHT11_Data DHT11_Read(void) {
  DHT11_Data dados = {0,0,0,0,1}; // Inicializa Error = 1
  uint8_t buffer[5] = {0,0,0,0,0}; 

  // Envia condição de Start
  DHT11_Start(); 

  // Salva o estado atual das interrupções e desabilita temporariamente
  uint8_t sreg_backup = SREG;
  cli();

  // Checa se a comunicação falhou
  if(!DHT11_CheckResponse()) {
    SREG = sreg_backup; // restaura backup do estado anterior
    return dados;
  }

  for (uint8_t i = 0; i < 5; i++) {
    buffer[i] = DHT11_ReadByte();
  }

  SREG = sreg_backup; // restaura backup do estado anterior

  // Verifica CheckSum
  uint8_t checksum = buffer[0] + buffer[1] + buffer[2] + buffer[3];

  if(checksum == buffer[4] && buffer[4] != 0) {
    dados.humidity_int    = buffer[0];
    dados.humidity_dec    = buffer[1];
    dados.temperature_int = buffer[2];
    dados.temperature_dec = buffer[3];
    dados.error = 0;  // Sucesso
  }
  return dados;
}
