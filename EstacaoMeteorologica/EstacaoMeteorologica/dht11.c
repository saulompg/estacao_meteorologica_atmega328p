/*  =-=-=-=-=-= Código comunicação com sensor DHT11 -=-=-=--=-=
 *
 *  Autor: Fábio Santos
 *
 *-=-=-=-=-=-=-=-=-=--=-=-=-=-=-=-=-=-=-=-=-=-=-=-==-=-=-=-=
 */

#include "dht11.h"

static void dht11_start(void) {

  DHT_DDR |= ( 1 << DHT_BIT); // atribuicao de saida 
  
  DHT_PORT &= ~(1 << DHT_BIT); // low para pc0
  _delay_ms(20);

  DHT_PORT |= (1<< DHT_BIT);
  _delay_us(30);
  
  DHT_DDR &= ~(1 << DHT_BIT);
  
}

static uint8_t dht11_check_resp(void) {
  uint16_t timeout = 250;
  
  // Espera o sensor baixar a tensao - 80us
  while (DHT_PIN & (1 << DHT_BIT)) {
    if (--timeout == 0) return 0;
    
  }
  // Espera o sensor subir a tensao - 80us 
  timeout = 250;
  while (!(DHT_PIN & (1 << DHT_BIT))) {
    if (--timeout == 0) return 0;
  }

  
  // Espera o sensor baixar novamente
  timeout = 250;
  while (DHT_PIN & (1 << DHT_BIT)) {
    if (--timeout == 0) return 0;
  }

  return 1; 


}

static uint8_t dht11_read_byte(void) {
    uint8_t result = 0;

    for(uint8_t i = 0; i < 8; i++) {
        
        // Espera o fim do preambulo - 50us de LOW
        while(!(DHT_PIN & (1 << DHT_BIT))); 
        // Espera para medir sinal de HIGH - faixa de corte (~28us ou ~70us)
        _delay_us(40); 

        // Verificar se esta HIGH apos 40us
        if (DHT_PIN & (1 << DHT_BIT)) {
            result |= (1 << (7-i)); 

            // Espera o bit 1 terminar
            while (DHT_PIN & (1 << DHT_BIT));
        }
       
    }
    return result;
}

DHT11_Data dht11_read(void) {
  DHT11_Data dados = {0,0,0,0,0,1}; // se nao chegar ao final, erro e 1
  uint8_t buffer[5];

  dht11_start(); 

  if(dht11_check_resp()) {

    for (int i = 0; i < 5; i++) {
      buffer[i] = dht11_read_byte();
    }

    if((buffer[0] + buffer[1] + buffer[2] + buffer[3]) == buffer[4]) {
      dados.hum_int = buffer[0];
      dados.hum_dec = buffer[1];
      dados.temp_int = buffer[2];
      dados.temp_dec = buffer[3];
      dados.checksum = buffer[4];
      dados.error = 0;  // entao erro e 0
      
    }
  }
  return dados;
}
