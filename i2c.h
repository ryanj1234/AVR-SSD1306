#ifndef __I2C_h__
#define __I2C_h__

#include <avr/io.h>

#define I2C_PRESCALER_1			0b00000000
#define I2C_PRESCALER_4			0b00000001
#define I2C_PRESCALER_16		0b00000010
#define I2C_PRESCALER_64		0b00000011
#define I2C_BRR_400_KHZ			12
#define I2C_status_mask			0xF8

void i2c_init(void);
uint8_t i2c_start(void);
uint8_t i2c_repeatedStart(void);
void i2c_stop(void);
uint8_t i2c_write(uint8_t);

#endif
