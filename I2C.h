#include <Arduino.h>
#include <Wire.h>

class I2C
{
	public:

		I2C( void );

		void setAdress( uint8_t _devAddress );

		uint8_t write( uint8_t address, uint8_t data );

		uint8_t write( uint8_t address, uint8_t *data, uint8_t length );

		uint8_t writeBit(uint8_t address, uint8_t pos, bool state);

		uint8_t read( uint8_t address );

		uint8_t read( uint8_t address, uint8_t *data, uint8_t nbytes );

	private:

		uint8_t devAddress; 

		// Used to check for errors in I2C communication
		uint16_t timeout = 1000; 
};
