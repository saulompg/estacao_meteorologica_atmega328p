/* ------------------------------------------------------------
 File: twi.h
 Author: Saulo
 Comments: Header para protocolo TWI (I2C)
------------------------------------------------------------ */

#ifndef TWI_H
#define	TWI_H

#include <avr/io.h>
#include <stdint.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define F_SCL 100000UL
#define PRESCALER 1
#define TWBR_VAL ((((F_CPU / F_SCL) / PRESCALER) - 16) / 2)

void TWI_Init();
void TWI_Start();
void TWI_Stop();
void TWI_Write(uint8_t data);
uint8_t TWI_Read_ACK(void);
uint8_t TWI_Read_NACK(void);

#endif	/* TWI_H */