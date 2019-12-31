#include <avr/io.h>
#include "i2c.h"

static __inline__ void _wait_for_flag_to_be_set(void)
{
	while(!(TWCR & _BV(TWINT)));
}

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

void i2c_init(void)
{
	// activate TWI in power reduction register
	PRR &= ~_BV(PRTWI);

	// disable TWI interrupts
	TWCR &= ~_BV(TWIE);

	// set prescaler bits
	TWSR |= I2C_PRESCALER_1;

	// set bit rate register
	TWBR = I2C_BRR_400_KHZ;
}

uint8_t i2c_start(void)
{
	_clear_flag_and_enable_w_start();

	_wait_for_flag_to_be_set();

	return _get_status();
}

__inline__ void i2c_stop(void)
{
	_clear_flag_and_enable_w_stop();
}

uint8_t i2c_write(uint8_t b)
{
	TWDR = b;

	_clear_flag_and_enable();

	_wait_for_flag_to_be_set();

	return _get_status();
}
