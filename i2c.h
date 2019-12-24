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
#define I2C_statusMask			0xF8
#define I2C_error 				0xFF

#define I2C_maxTries 			100

void I2C_init(void);
uint8_t I2C_start(void);
uint8_t I2C_repeatedStart(void);
void I2C_stop(void);
uint8_t I2C_write(uint8_t slaveAddr);
uint8_t I2C_readNack(uint8_t *dataBuf);
uint8_t I2C_readAck(uint8_t *dataBuf);
uint8_t I2C_readByte(uint8_t reg_addr, uint8_t dataRegister, uint8_t *dataByte);
uint8_t I2C_writeByte(uint8_t reg_addr, uint8_t dataRegister, uint8_t dataByte);

#endif
