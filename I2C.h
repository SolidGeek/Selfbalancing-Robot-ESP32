#include <Wire.h>
#include <Arduino.h>

class I2C
{
	public:

		I2C( uint8_t _deviceAddress );

		uint8_t write( uint8_t address, uint8_t data, bool sendStop );

		uint8_t write( uint8_t address, uint8_t *data, uint8_t length, bool sendStop );

		uint8_t read(uint8_t address, uint8_t *data, uint8_t nbytes );

    uint8_t read8(uint8_t address);

	private:

		uint8_t deviceAddress = 0x68; 

		// Used to check for errors in I2C communication
		uint16_t timeout = 1000; 
};
