#include <avr/io.h>
#include "i2c.h"

// #define I2C_clearFlagAndEnable() 			TWCR = (1 << TWINT)|(1 << TWEN)
#define I2C_clearFlagAndEnableWithStart() 	TWCR = (1 << TWINT)|(1 << TWEN)|(1 << TWSTA)
#define I2C_clearFlagAndEnableWithStop()	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO)
#define I2C_clearFlagAndEnableWithAck()		TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA)
#define I2C_waitForFlagToBeSet() 			while(!(TWCR & (1 << TWINT)))
#define	I2C_getStatus()						(TWSR & I2C_statusMask)

static void I2C_clearFlagAndEnable(void) { TWCR = _BV(TWINT) | _BV(TWEN); }

/*************************************************************************
 Initializes I2C. Must be called to use I2C. Sets bit rate to 400 by default
*************************************************************************/
void I2C_init(void)
{
	// activate TWI in power reduction register
	PRR &= ~(1 << PRTWI);

	// disable TWI interrupts
	TWCR &= ~(1 << TWIE);

	// set prescaler bits
	TWSR |= I2C_PRESCALER_1;

	// set bit rate register
	TWBR = I2C_BRR_400_KHZ;
}

/*************************************************************************
 Sends start condition on bus. Returns I2C status code
*************************************************************************/
uint8_t I2C_start(void)
{
	// TWEN is set to enable TWI, TWINT must be cleared (set to 1), and TWSTA is set to send a start
	I2C_clearFlagAndEnableWithStart();

	// wait for TWINT flag
	I2C_waitForFlagToBeSet();

	// return status code
	return I2C_getStatus();
}

/*************************************************************************
 Sends repeated start condition on bus. Returns I2C status code
*************************************************************************/
uint8_t I2C_repeatedStart(void)
{
	// TWEN is set to enable TWI, TWINT must be cleared (set to 1), and TWSTA is set to send a start
	I2C_clearFlagAndEnableWithStart();

	// wait for flag
	I2C_waitForFlagToBeSet();

	// return status code
	return I2C_getStatus();
}

/*************************************************************************
 Sends stop condition on bus. No return
*************************************************************************/
void I2C_stop(void)
{
	// set bits in control register to send stop condition
	I2C_clearFlagAndEnableWithStop();
}

/*************************************************************************
 Writes data to bus. Returns I2C status code
*************************************************************************/
uint8_t I2C_write(uint8_t slaveAddr)
{
	// load slave address into data register
	TWDR = slaveAddr;

	// clear flag and enable
	I2C_clearFlagAndEnable();

	// wait for flag to be set
	I2C_waitForFlagToBeSet();

	// return status code
	return I2C_getStatus();
}

/*************************************************************************
 Reads data byte into location of input pointer. Transmits Nack afterwards
*************************************************************************/
uint8_t I2C_readNack(uint8_t *dataBuf)
{
	// clear flag and enable
	I2C_clearFlagAndEnable();

	// wait for flag to be set
	I2C_waitForFlagToBeSet();

	// read data register into buffer
	*dataBuf = TWDR;

	// return status code
	return I2C_getStatus();
}

/*************************************************************************
 Reads data byte into location of input pointer. Transmits Ack afterwards
*************************************************************************/
uint8_t I2C_readAck(uint8_t *dataBuf)
{
	// clear flag and enable
	I2C_clearFlagAndEnableWithAck();

	// wait for flag to be set
	I2C_waitForFlagToBeSet();

	// read data register into buffer
	*dataBuf = TWDR;

	// return status code
	return I2C_getStatus();
}

#define __DEBUG 		1
#define	ADDRW(X)		X
#define	ADDRR(X)		X | 0x01
uint8_t I2C_readByte(uint8_t reg_addr, uint8_t dataRegister, uint8_t *dataByte)
{
	// status flag
	uint8_t I2C_status;

	// send start condition
	if(!(I2C_status = I2C_start()))
	{
		// return error
		return I2C_error;
	}

	// send slave address + W
	if((I2C_status = I2C_write(ADDRW(reg_addr))) != I2C_slaWack)
	{
		// return error
		return I2C_error;
	}

	// send data register address
	if((I2C_status = I2C_write(dataRegister)) != I2C_dataAck)
	{
		// return error
		return I2C_error;
	}

	// Send repeat start
	if((I2C_status = I2C_repeatedStart()) != I2C_repeatedStartCode)
	{
		// return error
		return I2C_error;
	}

	// Tell slave go in send mode
	if((I2C_status = I2C_write(ADDRR(reg_addr))) != I2C_slaRack)
	{
		// return error
		return I2C_error;
	}

	// Read data
	if((I2C_status = I2C_readNack(dataByte)) != I2C_dataNackRet)
	{
		// return error
		return I2C_error;
	}

	// Send stop
	I2C_stop();

	// return success
	return 0x00;
}

uint8_t I2C_writeByte(uint8_t reg_addr, uint8_t dataRegister, uint8_t dataByte)
{
	// status flag
	uint8_t I2C_status;

	// send start condition
	if(!(I2C_status = I2C_start()))
	{
		// return error
		return I2C_error;
	}

	// send slave address + W
	if((I2C_status = I2C_write(ADDRW(reg_addr))) != I2C_slaWack)
	{
		// return error
		return I2C_error;
	}

	// send data register address
	if((I2C_status = I2C_write(dataRegister)) != I2C_dataAck)
	{
		// return error
		return I2C_error;
	}

	// write data to register address
	if((I2C_status = I2C_write(dataByte)) != I2C_dataAck)
	{
		// return error
		return I2C_error;
	}

	// send stop condition
	I2C_stop();

	// return success
	return 0x00;
}
