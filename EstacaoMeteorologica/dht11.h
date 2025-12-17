/* ------------------------------------------------------------
 File: dht11.h
 Author: Fábio Santos
 Comments: Header para comunicação com DHT11
------------------------------------------------------------ */

#ifndef DHT11_H
#define DHT11_H

#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>

#define DHT_PORT PORTC
#define DHT_DDR DDRC
#define DHT_PIN PINC
#define DHT_BIT PC0

#define DHT11_MAX_TIMEOUT 200

// Estrutura de dados
typedef struct {
  uint8_t humidity_int;
  uint8_t humidity_dec;
  uint8_t temperature_int;
  uint8_t temperature_dec;
  uint8_t checksum;
  uint8_t error;            // se Erro, error = 1
} DHT11_Data;

DHT11_Data DHT11_Read(void);

#endif
