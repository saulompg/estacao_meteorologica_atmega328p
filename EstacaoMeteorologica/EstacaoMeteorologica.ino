/* ---------------------------------------------
  ANOTAÇÕES GERAIS SORBE O PROTOCOLO TWI
  TWBR (TWI Bit Rate Register): Define a velocidade do Clock (SCL)
  TWSR (TWI Status Register): Define o Prescaler (Bits TWPS0 e TWPS1) e Armazena o Status das operações
  TWCR (TWI Control Register): Controla as funcionalidades do protocolo
    TWINT (Interrupt Flag): Sinaliza quando o hardware terminou de enviar um byte
    TWEN (Enable): Liga o módulo I2C
    TWSTA (Start): Manda gerar um sinal de Start
    TWSTO (Stop): Manda gerar um sinal de Stop
    TWEA (Enable Acknowledge): Se ligado, o ATmega responderá com ACK qual alguem falar com ele
  TWDR (TWI Data Register): Buffer de 8 bits que armazena os dados a serem enviados e os dados recebidos

  BMP280 I2C Driver for ATmega328p
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
#include <stdint.h>

// CONFIGURAÇÕES I2C
#define F_SCL 100000UL // Clock I2C de 100kHz
#define PRESCALER 1
// Cálculo do Bit Rate (Fórmula do Datasheet)
#define TWBR_VAL ((((F_CPU / F_SCL) / PRESCALER) - 16) / 2)

// Endereço do BMP280
#define BMP280_ADDR      0x76 // Endereço I2C (7 bits)
#define BMP280_REG_ID    0xD0 // Registrador de ID do chip (deve retornar 0x58)
#define BMP280_REG_CTRL  0xF4 // Controle de medição
#define BMP280_REG_DATA  0xF7 // Início dos dados (Pressão MSB)
#define BMP280_REG_CALIB 0x88 // Início dos dados de calibração

// --- Variáveis Globais ---
int32_t t_fine;

// Estrutura para armazenar os parâmetros de calibração do BMP280
struct BMP280_Calib {
  uint16_t dig_T1;
  int16_t  dig_T2;
  int16_t  dig_T3;
  uint16_t dig_P1;
  int16_t  dig_P2;
  int16_t  dig_P3;
  int16_t  dig_P4;
  int16_t  dig_P5;
  int16_t  dig_P6;
  int16_t  dig_P7;
  int16_t  dig_P8;
  int16_t  dig_P9;
} calib;

// ----------------------------------------
// ESCOPO DAS FUNÇÕES
// ----------------------------------------
void mcu_init(void);
void TWI_Init(void);
void TWI_Start(void);
void TWI_Stop(void);
void TWI_Write(uint8_t data);
uint8_t TWI_Read_ACK(void);
uint8_t TWI_Read_NACK(void);
void BMP280_WriteReg(uint8_t reg, uint8_t value);
void BMP280_Init(void);
void BMP280_ReadRaw(void);
uint16_t BMP280_Read16LE(uint8_t reg);
void BMP280_ReadCalibrationParams(void);
int32_t BMP280_Compensate_T(int32_t adc_T);
uint32_t BMP280_Compensate_P(int32_t adc_P);

// ----------------------------------------
// ENTRADA DO PROGRAMA PRINCIPAL
// ----------------------------------------
int main(void) {
  mcu_init();

  int32_t raw_temp, raw_press;
  int32_t temp_final;
  uint32_t press_final;
  
  while (1) {
    // --- Leitura dos Dados Brutos (Burst Read) ---
    TWI_Start();
    TWI_Write(BMP280_ADDR << 1);
    TWI_Write(0xF7); // Ponteiro no início dos dados

    TWI_Start(); // Repeated Start para Leitura
    TWI_Write((BMP280_ADDR << 1) | 1);

    uint8_t p_msb = TWI_Read_ACK();
    uint8_t p_lsb = TWI_Read_ACK();
    uint8_t p_xlsb = TWI_Read_ACK();
    uint8_t t_msb = TWI_Read_ACK();
    uint8_t t_lsb = TWI_Read_ACK();
    uint8_t t_xlsb = TWI_Read_NACK();
    TWI_Stop();

    // Montagem dos inteiros de 20 bits
    raw_press = ((int32_t)p_msb << 12) | ((int32_t)p_lsb << 4) | (p_xlsb >> 4);
    raw_temp  = ((int32_t)t_msb << 12) | ((int32_t)t_lsb << 4) | (t_xlsb >> 4);

    // --- Conversão Matemática ---
    temp_final = BMP280_Compensate_T(raw_temp); 
    press_final = BMP280_Compensate_P(raw_press);

    // led acende se temperatura estiver acima de 35°C
    if (temp_final >= 3500) {
      PORTB |= (1<<PB0);
    } else {
      PORTB &= ~(1<<PB0);
    }

    // led acende se pressão cair para 1/2 ATM (hPa)
    if (press_final > 50600) {
      PORTB &= ~(1<<PB1);
    } else {
      PORTB |= (1<<PB1);
    }

    _delay_ms(1000);
  }
  return 0;
}

// --------------------------------------------------------
// CONFIGURAÇÃO DE BAIXO NÍVEL
// --------------------------------------------------------
void mcu_init(void) {
  // 1. Desativa Watchdog
  wdt_disable();

  // 2. Desabilita Interrupções Globais
  cli();

  // 3. Configura Clock Prescaler para 1
  CLKPR = (1 << CLKPCE); // Habilita a mudança
  CLKPR = 0x00;          // Define divisor como 1 (Clock total)

  // 4. Inicializar Portas
  DDRB  = 0xFF; 
  PORTB = 0x00;

  // 5. Configurar periféricos
  TWI_Init();     // Configura protocolo I2C
  _delay_ms(100); // Tempo para o sensor ligar
  
  // Inicialização do Sensor
  BMP280_Init();

  BMP280_ReadCalibrationParams();

  // 6. Habilita IG
  sei();
}

// --------------------------------------------------------
// FUNÇÕES PARA IMPLEMENTAR O TWI (Two Wires Interface)
// --------------------------------------------------------
void TWI_Init(void) {
	TWSR = 0x00;                // Prescaler = 1
  TWBR = (uint8_t)TWBR_VAL;   // Configura a velocidade
	TWCR = (1<<TWEN);           // Habilita o módulo TWI
}

void TWI_Start(void) {
  // Envia Start Condition
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
  // Espera a TWINT Flag. Indica que o hardware terminou
	while (!(TWCR & (1 << TWINT)));
}

void TWI_Stop(void) {
  // Envia Stop Condition
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
}

void TWI_Write(uint8_t data) {
	// Armazena dado no buffer
  TWDR = data;
  // Inicia transmissão
	TWCR = (1<<TWINT) | (1<<TWEN);
  // Espera terminar e receber o ACK/NACK do Secundário
	while (!(TWCR & (1<<TWINT)));
}

// Lê um byte e envia ACK (pedindo mais dados)
uint8_t TWI_Read_ACK(void) {
  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
  while (!(TWCR & (1<<TWINT)));
  return TWDR;
}

// Lê um byte e envia NACK (dizendo "chega, parei")
uint8_t TWI_Read_NACK(void) {
  TWCR = (1<<TWINT) | (1<<TWEN);
  while (!(TWCR & (1<<TWINT)));
  return TWDR;
}

// --------------------------------------------------------
// FUNÇÕES AUXILIARES PARA BMP280
// --------------------------------------------------------
void BMP280_Init(void) {
  // Configura registrador 0xF4 (ctrl_meas)
  // Osrs_t x1 (001), Osrs_p x1 (001), Mode Normal (11)
  // Binário: 001 001 11 = 0x27
  BMP280_WriteReg(BMP280_REG_CTRL, 0x27);
}

void BMP280_WriteReg(uint8_t reg, uint8_t value) {
  TWI_Start();
  TWI_Write(BMP280_ADDR << 1); // Endereço + Write (0)
  TWI_Write(reg);              // Ponteiro do registrador
  TWI_Write(value);            // Valor a escrever
  TWI_Stop();
}

uint16_t BMP280_Read16LE(uint8_t reg) {
  TWI_Start();
  TWI_Write(BMP280_ADDR << 1);
  TWI_Write(reg);
  
  TWI_Start(); // Repeated Start
  TWI_Write((BMP280_ADDR << 1) | 1);
  
  uint8_t lsb = TWI_Read_ACK();
  uint8_t msb = TWI_Read_NACK();
  TWI_Stop();
  
  return (uint16_t)((msb << 8) | lsb);
}

void BMP280_ReadCalibrationParams(void) {
  calib.dig_T1 = (uint16_t) BMP280_Read16LE(0x88);
  calib.dig_T2 = (int16_t)  BMP280_Read16LE(0x8A);
  calib.dig_T3 = (int16_t)  BMP280_Read16LE(0x8C);
  
  calib.dig_P1 = (uint16_t) BMP280_Read16LE(0x8E);
  calib.dig_P2 = (int16_t)  BMP280_Read16LE(0x90);
  calib.dig_P3 = (int16_t)  BMP280_Read16LE(0x92);
  calib.dig_P4 = (int16_t)  BMP280_Read16LE(0x94);
  calib.dig_P5 = (int16_t)  BMP280_Read16LE(0x96);
  calib.dig_P6 = (int16_t)  BMP280_Read16LE(0x98);
  calib.dig_P7 = (int16_t)  BMP280_Read16LE(0x9A);
  calib.dig_P8 = (int16_t)  BMP280_Read16LE(0x9C);
  calib.dig_P9 = (int16_t)  BMP280_Read16LE(0x9E);
}

int32_t BMP280_Compensate_T(int32_t adc_T) {
  int32_t var1, var2, T;
  
  var1 = ((((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1))) * ((int32_t)calib.dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)calib.dig_T1))) >> 12) * ((int32_t)calib.dig_T3)) >> 14;
  
  t_fine = (int32_t)var1 + var2; // Atualiza variável global para uso na pressão
  
  T = (t_fine * 5 + 128) >> 8;
  return T; 
}

uint32_t BMP280_Compensate_P(int32_t adc_P) {
  int32_t var1, var2;
  uint32_t p;
  
  // O cálculo da pressão depende do t_fine (temperatura atual)
  var1 = (((int32_t)t_fine)>>1) - (int32_t)64000;
  var2 = (((var1>>2) * (var1>>2) >> 11) * ((int32_t)calib.dig_P6));
  var2 = var2 + ((var1 * ((int32_t)calib.dig_P5)) << 1);
  var2 = (var2>>2) + (((int32_t)calib.dig_P4) << 16);
  var1 = (((calib.dig_P3 * (((var1>>2) * (var1>>2)) >> 13)) >> 3) + ((((int32_t)calib.dig_P2) * var1) >> 1)) >> 18;
  var1 = ((((32768 + var1)) * ((int32_t)calib.dig_P1)) >> 15);
  
  if (var1 == 0) {
    return 0; // Evita divisão por zero
  }
  
  p = (((uint32_t) (((int32_t)1048576) - adc_P) - (var2>>12))) * 3125;
  if (p < 0x80000000) {
    p = (p << 1) / ((uint32_t) var1);
  } else {
    p = (p / (uint32_t)var1) * 2;
  }
  var1 = (((int32_t)calib.dig_P9) * ((int32_t) (((p>>3) * (p>>3))>>13)))>>12;
  var2 = (((int32_t) (p>>2)) * ((int32_t)calib.dig_P8))>>13;
  
  p = (uint32_t) ((int32_t)p + ((var1 + var2 + calib.dig_P7) >> 4));
  
  return p;
}