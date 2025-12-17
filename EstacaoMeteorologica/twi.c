/* ------------------------------------------------------------
 File: twi.c
 Author: Saulo
 Comments: Implementacao do protocolo TWI (I2C)

 ANOTAÇÕES GERAIS SORBE O PROTOCOLO TWI
    TWBR (TWI Bit Rate Register): Define a velocidade do Clock (SCL)
    TWSR (TWI Status Register): Define o Prescaler (Bits TWPS0 e TWPS1) e Armazena o Status das operações
    TWCR (TWI Control Register): Controla as funcionalidades do protocolo
      TWINT (Interrupt Flag): Sinaliza quando o hardware terminou de enviar um byte
      TWEN (Enable): Liga o módulo I2C
      TWSTA (Start): Manda gerar um sinal de Start
      TWSTO (Stop): Manda gerar um sinal de Stop
      TWEA (Enable Acknowledge): Se ligado, o ATmega responderá com ACK quando alguem falar com ele
    TWDR (TWI Data Register): Buffer de 8 bits que armazena os dados a serem enviados e os dados recebidos
------------------------------------------------------------ */

#include "twi.h"
#include <util/delay.h>

void TWI_Init(void) {
  TWSR = 0x00;                // Prescaler = 1
  TWBR = (uint8_t) TWBR_VAL;  // Configura a velocidade
  TWCR = (1 << TWEN);         // Habilita o modulo TWI
}

uint8_t TWI_Start(void) {
  // Envia Start Condition
  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);

  uint16_t timeout = TWI_TIMEOUT_MAX;

  // Espera a TWINT Flag. Indica que o hardware terminou
  while (!(TWCR & (1 << TWINT))) {
    _delay_us(1);
    if(--timeout == 0) return 1; // FALHA
  }
  return 0; // SUCESSO
}

void TWI_Stop(void) {
  // Envia Stop Condition
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}

uint8_t TWI_Write(uint8_t data) {
  // Armazena dado no buffer
  TWDR = data;

  uint16_t timeout = TWI_TIMEOUT_MAX;

  // Inicia a transmissão
  TWCR = (1 << TWINT) | (1 << TWEN);
  // Espera terminar e receber o ACK/NACK
  while (!(TWCR & (1 << TWINT))) {
    _delay_us(1);
    if (--timeout == 0) return 1;
  }
  return 0;
}

uint8_t TWI_Read_ACK(void) {
  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);

  uint16_t timeout = TWI_TIMEOUT_MAX;

  while (!(TWCR & (1<<TWINT))) {
    _delay_us(1);
    if (--timeout == 0) return 0xFF; // retorno seguro em caso de timeout
  }
  return TWDR;
}

uint8_t TWI_Read_NACK(void) {
  TWCR = (1<<TWINT) | (1<<TWEN);

  uint16_t timeout = TWI_TIMEOUT_MAX;

  while (!(TWCR & (1<<TWINT))) {
    _delay_us(1);
    if (--timeout == 0) return 0xFF; // retorno seguro em caso de timeout
  }
  return TWDR;
}

