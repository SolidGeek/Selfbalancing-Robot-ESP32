#include "I2C.h"

I2C::I2C(uint8_t _deviceAddress)
{
	deviceAddress = _deviceAddress;
}

uint8_t I2C::write(uint8_t address, uint8_t data, bool sendStop)
{
	return write(address, &data, 1, sendStop); // Returns 0 on success
}

uint8_t I2C::write(uint8_t address, uint8_t *data, uint8_t length, bool sendStop)
{
	// Begin communication on the I2C address
	Wire.beginTransmission(deviceAddress);	

	// First write the address to update
	Wire.write(address);
	// Next write the data
	Wire.write(data, length);

	// End communication, returns 0 on success
	uint8_t response = Wire.endTransmission(sendStop);

	return response;
}

uint8_t I2C::read8(uint8_t address)
{
    uint8_t value;

    Wire.beginTransmission(deviceAddress);
    Wire.write(address);
 
    Wire.endTransmission();

    Wire.beginTransmission(deviceAddress);
    Wire.requestFrom(deviceAddress, 1);
    
    while(!Wire.available()) {};
    
    value = Wire.read();
    Wire.endTransmission();

    return value;
}

uint8_t I2C::read(uint8_t address, uint8_t *data, uint8_t nbytes)
{
	uint32_t timeOutTimer;

	Wire.beginTransmission(deviceAddress);
	Wire.write(address);
  uint8_t response = Wire.endTransmission(); 

  if (response) {
    Serial.println("End transmission failed");
    return response;
  }


  Wire.beginTransmission(deviceAddress);
	Wire.requestFrom(deviceAddress, nbytes, (uint8_t)true); // Send a repeated start and then release the bus after reading

	for (uint8_t i = 0; i < nbytes; i++)
	{
		if (Wire.available())
			data[i] = Wire.read();
		else {
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
