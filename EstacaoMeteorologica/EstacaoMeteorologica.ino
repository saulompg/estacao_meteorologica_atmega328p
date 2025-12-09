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

// --- Configurações I2C ---
#define F_SCL 100000UL // Clock I2C de 100kHz
#define PRESCALER 1
#define TWBR_VAL ((((F_CPU / F_SCL) / PRESCALER) - 16) / 2) // Cálculo do Bit Rate (Fórmula do Datasheet)

// --- Configurações BMP280 ---
#define BMP280_ADDR      0x76 // Endereço I2C (7 bits)
#define BMP280_REG_CTRL  0xF4 // Controle de medição
#define BMP280_REG_DATA  0xF7 // Início dos dados (Pressão MSB)
#define BMP280_REG_CALIB 0x88 // Início dos dados de calibração

// --- Limites de Alarme ---
#define TEMP_THRESHOLD   3500   // 35.00 Graus Celsius
#define PRESS_THRESHOLD  50600  // 506.00 hPa

// --- Variáveis Globais ---
int32_t t_fine;
volatile uint8_t flag_ler_sensor = 0;

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

// --- Protótipos ---
void MCU_Init(void);
void TWI_Init(void);
void TWI_Start(void);
void TWI_Stop(void);
void TWI_Write(uint8_t data);
uint8_t TWI_Read_ACK(void);
uint8_t TWI_Read_NACK(void);
void BMP280_WriteReg(uint8_t reg, uint8_t value);
void BMP280_ReadCalibrationBurst(void);
int32_t BMP280_Compensate_T(int32_t adc_T);
uint32_t BMP280_Compensate_P(int32_t adc_P);

// ----------------------------------------
// PROGRAMA PRINCIPAL
// ----------------------------------------
int main(void) {
  MCU_Init();

  // Inicialização do Sensor (Normal mode, Oversampling x1)
  // Osrs_t x1 (001), Osrs_p x1 (001), Mode Normal (11)
  // Binário: 001 001 11 = 0x27
  BMP280_WriteReg(BMP280_REG_CTRL, 0x27); 
  
  // Leitura da calibração (Burst Read)
  BMP280_ReadCalibrationBurst();

  int32_t raw_temp, raw_press;
  int32_t temp_final;
  uint32_t press_final;
  
  while (1) {

    // Verifica se o Timer1 mandou executar (passou 1 segundo)
    if (flag_ler_sensor) {
      flag_ler_sensor = 0; // Limpa a flag imediatamente
    }

    // --- Leitura dos Dados Brutos (Burst Read) ---
    TWI_Start();
    TWI_Write(BMP280_ADDR << 1);
    TWI_Write(BMP280_REG_DATA);   // Ponteiro no início dos dados

    TWI_Start(); // Restart
    TWI_Write((BMP280_ADDR << 1) | 1); // Modo Leitura

    // Lê 6 bytes em sequência
    uint8_t p_msb = TWI_Read_ACK();
    uint8_t p_lsb = TWI_Read_ACK();
    uint8_t p_xlsb = TWI_Read_ACK();
    uint8_t t_msb = TWI_Read_ACK();
    uint8_t t_lsb = TWI_Read_ACK();
    uint8_t t_xlsb = TWI_Read_NACK();   // Último byte = NACK
    TWI_Stop();

    // Montagem dos inteiros de 20 bits
    raw_press = ((int32_t)p_msb << 12) | ((int32_t)p_lsb << 4) | (p_xlsb >> 4);
    raw_temp  = ((int32_t)t_msb << 12) | ((int32_t)t_lsb << 4) | (t_xlsb >> 4);

    // Compensação
    temp_final = BMP280_Compensate_T(raw_temp); 
    press_final = BMP280_Compensate_P(raw_press);

    // EXEMPLO DE APLICAÇÃO
    // LED PB0: Temperatura Alta
    if (temp_final >= TEMP_THRESHOLD) PORTB |= (1<<PB0);
    else PORTB &= ~(1<<PB0);

    // LED PB1: Pressão Baixa
    if (press_final > PRESS_THRESHOLD) PORTB &= ~(1<<PB1);
    else PORTB |= (1<<PB1);

    _delay_ms(1000);
  }
  return 0;
}

// ----------------------------------------
// CONFIGURAÇÃO DO TIMER 1 (1 Segundo)
// ----------------------------------------
void Timer1_Init(void) {
    // Configura Modo CTC (Clear Timer on Compare Match)
    // TCCR1B: WGM12 = 1 (Tabela 16-4 do datasheet, p. 130)
    TCCR1B |= (1 << WGM12);

    // Define o valor de comparação para 1 segundo (1Hz)
    // Fórmula: (16.000.000 / (1024 * 1)) - 1 = 15624
    // Datasheet p. 132 (OCR1A)
    OCR1A = 15624;

    // Habilita Interrupção de Comparação A
    // TIMSK1: OCIE1A = 1 (Datasheet p. 133)
    TIMSK1 |= (1 << OCIE1A);

    // Inicia o Timer com Prescaler 1024
    // TCCR1B: CS12 = 1, CS10 = 1 (Tabela 16-5 do datasheet, p. 131)
    TCCR1B |= (1 << CS12) | (1 << CS10);
}

// Rotina de Serviço de Interrupção (ISR)
ISR(TIMER1_COMPA_vect) {
    // Esta função é chamada automaticamente pelo hardware a cada 1 segundo
    flag_ler_sensor = 1;
}

// --------------------------------------------------------
// CONFIGURAÇÃO DE BAIXO NÍVEL
// --------------------------------------------------------
void MCU_Init(void) {
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

  // 5. Configura protocolo I2C
  TWI_Init(); 
  _delay_ms(100);

  // 6. Habilita Interrupção Global
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

// Lê um byte e solicita mais dados
uint8_t TWI_Read_ACK(void) {
  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
  while (!(TWCR & (1<<TWINT)));
  return TWDR;
}

// Lê um byte e encerra
uint8_t TWI_Read_NACK(void) {
  TWCR = (1<<TWINT) | (1<<TWEN);
  while (!(TWCR & (1<<TWINT)));
  return TWDR;
}

// --------------------------------------------------------
// FUNÇÕES AUXILIARES PARA BMP280
// --------------------------------------------------------
void BMP280_WriteReg(uint8_t reg, uint8_t value) {
  TWI_Start();
  TWI_Write(BMP280_ADDR << 1); // Endereço + Write (0)
  TWI_Write(reg);              // Ponteiro do registrador
  TWI_Write(value);            // Valor a escrever
  TWI_Stop();
}

// Leitura em burst dos bytes de calibração
void BMP280_ReadCalibrationBurst(void) {
  uint8_t buf[24];

  TWI_Start();
  TWI_Write(BMP280_ADDR << 1);
  TWI_Write(BMP280_REG_CALIB);

  TWI_Start();
  TWI_Write((BMP280_ADDR << 1) | 1);

  // Lê os primeiros 23 bytes com ACK
  for (int i = 0; i < 23; i++) {
    buf[i] = TWI_Read_ACK();
  }
  // Lê o último byte e responde com NACK
  buf[23] = TWI_Read_NACK();
  TWI_Stop();

  // Reconstrói a estrutura (Little Endian)
  calib.dig_T1 = (buf[1] << 8) | buf[0];    // 0x88
  calib.dig_T2 = (buf[3] << 8) | buf[2];    // 0x8A
  calib.dig_T3 = (buf[5] << 8) | buf[4];    // 0x8C
  calib.dig_P1 = (buf[7] << 8) | buf[6];    // 0x8E
  calib.dig_P2 = (buf[9] << 8) | buf[8];    // 0x90
  calib.dig_P3 = (buf[11] << 8) | buf[10];  // 0x92
  calib.dig_P4 = (buf[13] << 8) | buf[12];  // 0x94
  calib.dig_P5 = (buf[15] << 8) | buf[14];  // 0x96
  calib.dig_P6 = (buf[17] << 8) | buf[16];  // 0x98
  calib.dig_P7 = (buf[19] << 8) | buf[18];  // 0x9A
  calib.dig_P8 = (buf[21] << 8) | buf[20];  // 0x9C
  calib.dig_P9 = (buf[23] << 8) | buf[22];  // 0x9E
}

// Função matemática de compensação 32 bits para Temperatura
int32_t BMP280_Compensate_T(int32_t adc_T) {
  int32_t var1, var2, T;
  
  var1 = ((((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1))) * ((int32_t)calib.dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)calib.dig_T1))) >> 12) * ((int32_t)calib.dig_T3)) >> 14;
  
  t_fine = (int32_t)var1 + var2; // Atualiza variável global para uso na pressão
  
  T = (t_fine * 5 + 128) >> 8;
  return T; 
}

// Função matemática de compensação 32 bits para Pressão
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