#include "I2C.h"

I2C::I2C()
{
	Wire.begin();
	Wire.setClock(400000UL); // Set I2C frequency to 400kHz
}

void I2C::setAdress( uint8_t address ){
	devAddress = address;
}

uint8_t I2C::write(uint8_t address, uint8_t data)
{
	uint8_t response = write(address, &data, 1); // Returns 0 on success
  
  return response;
}

uint8_t I2C::write(uint8_t address, uint8_t *data, uint8_t length)
{

	// Begin communication on the I2C address
	Wire.beginTransmission(devAddress);	

	// First write (select) the register to update
	Wire.write(address);

	// Next write the data
	Wire.write(data, length);

	// End communication, returns 0 on success
	return Wire.endTransmission();

}

uint8_t I2C::read(uint8_t address){
	uint8_t data;
	read(address, &data, 1);

	return data;
}

uint8_t I2C::read(uint8_t address, uint8_t *data, uint8_t nbytes)
{
	uint32_t timeOutTimer;
	uint8_t response;

	// Begin communication on the I2C address
	Wire.beginTransmission(devAddress);

	// First write (select) the register to read from
	Wire.write(address);
	response = Wire.endTransmission(); 

	if (response) {
		return response;
	}

	// Read n bytes from the I2C
	Wire.beginTransmission(devAddress);
	Wire.requestFrom(devAddress, nbytes, true); 

	for (uint8_t i = 0; i < nbytes; i++)
	{
		if (Wire.available())
		{
			data[i] = Wire.read();
		}
		else
		{
			timeOutTimer = micros();

			while (((micros() - timeOutTimer) < timeout) && !Wire.available());

			if (Wire.available())
				data[i] = Wire.read();
			else {
				return 5; // This error value is not already taken by endTransmission
			}

		}
	}

	return 0; // Success
}


uint8_t I2C::writeBit(uint8_t address, uint8_t pos, bool state)
{
    uint8_t value;
    value = read(address);

    if (state)
    {
        value |= (1 << pos);
    } else 
    {
        value &= ~(1 << pos);
    }

    return write(address, value);
}
