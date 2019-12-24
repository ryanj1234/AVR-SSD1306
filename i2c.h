#ifndef __I2C_h__
#define __I2C_h__

#include <avr/io.h>

/***************************** Configuration *****************************/
#define I2C_PRESCALER_1			0b00000000
#define I2C_PRESCALER_4			0b00000001
#define I2C_PRESCALER_16		0b00000010
#define I2C_PRESCALER_64		0b00000011
#define I2C_BRR_400_KHZ			12

/****************************** Status codes *****************************/
#define I2C_success				0x00
#define I2C_startCode 			0x08
#define I2C_repeatedStartCode 	0x10
#define I2C_slaWack 			0x18
#define I2C_dataAck 			0x28
#define I2C_slaRack 			0x40
#define I2C_dataAckRet 			0x50
#define I2C_dataNackRet 		0x58
#define I2C_status_mask			0xF8
#define I2C_error 				0xFF

#define I2C_maxTries 			100

void i2c_init(void);
uint8_t i2c_start(void);
uint8_t i2c_repeatedStart(void);
void i2c_stop(void);
uint8_t i2c_write(uint8_t slaveAddr);

#endif
