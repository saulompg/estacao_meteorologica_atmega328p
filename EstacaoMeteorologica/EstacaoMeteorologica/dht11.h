/*  =-=-=-=-=-= Header para comunicação com DHT11 -=-=-=--=-=
 *
 *  Autor: Fábio Santos
 *
 *-=-=-=-=-=-=-=-=-=--=-=-=-=-=-=-=-=-=-=-=-=-=-=-==-=-=-=-=
 */


#ifndef DHT11_H
#define DHT11_H
#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>

// -=-=-=-= Configurações de pino -=-=-=-=-

#define DHT_PORT PORTC
#define DHT_DDR DDRC
#define DHT_PIN PINC
#define DHT_BIT 0 //Pino PCO


// =-=-=-=- Estrutura de dados -=-=-==-=- 

typedef struct {
  uint8_t hum_int;
  uint8_t hum_dec;
  uint8_t temp_int;
  uint8_t temp_dec;
  uint8_t checksum;
  uint8_t error; // se Erro, error = 1
  
}DHT11_Data;

void dht11_init(void);
DHT11_Data dht11_read(void);

#endif
