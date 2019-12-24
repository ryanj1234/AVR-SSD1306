#include <avr/io.h>
#include "i2c.h"

#define i2c_waitForFlagToBeSet() 			while(!(TWCR & (1 << TWINT)))

static __inline__ void _clear_flag_and_enable(void) 
{ 
	TWCR = _BV(TWINT) | _BV(TWEN); 
}

static __inline__ void _clear_flag_and_enable_w_start(void)
{
	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTA);
}

static __inline__ void _clear_flag_and_enable_w_stop(void)
{
	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO);
}

static __inline__ uint8_t _get_status(void)
{
	return TWSR & I2C_status_mask;
}


/*************************************************************************
 Initializes i2c. Must be called to use i2c. Sets bit rate to 400 by default
*************************************************************************/
void i2c_init(void)
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

uint8_t i2c_start(void)
{
	// TWEN is set to enable TWI, TWINT must be cleared (set to 1), and TWSTA is set to send a start
	_clear_flag_and_enable_w_start();

	// wait for TWINT flag
	i2c_waitForFlagToBeSet();

	// return status code
	return _get_status();
}

void i2c_stop(void)
{
	// set bits in control register to send stop condition
	_clear_flag_and_enable_w_stop();
}

uint8_t i2c_write(uint8_t slaveAddr)
{
	// load slave address into data register
	TWDR = slaveAddr;

	// clear flag and enable
	_clear_flag_and_enable();

	// wait for flag to be set
	i2c_waitForFlagToBeSet();

	// return status code
	return _get_status();
}
