#include "MPU6050.h"

MPU6050::MPU6050(void){}

bool MPU6050::begin(){

	port.setAdress( MPU6050_ADRESS );

	delay(100);

	if (port.read(MPU6050_REG_WHO_AM_I) != MPU6050_ADRESS){
		return false;
	}

	setSleepEnabled( false ); // Turn of sleep mode before any other registers is configured!!!
	// setClockSource( MPU6050_CLOCK_PLL_XGYRO );
	setAccelRange( MPU6050_RANGE_4G );
	setGyroRange( MPU6050_SCALE_250DPS );
  setDLPF(4); // Enable the digital low pass filter on all the raw outputs
  
	this->ready = true;

	return true;

}

bool MPU6050::isReady( void ){
	return this->ready;
}

void MPU6050::setClockSource( uint8_t source )
{
    uint8_t value;
    
    value = port.read(MPU6050_REG_PWR_MGMT_1);
    
    value &= 0b11111000;
    value |= source;
 
    port.write(MPU6050_REG_PWR_MGMT_1, value);
}

void MPU6050::setAccelRange( uint8_t range )
{
    uint8_t value;

    value = port.read(MPU6050_REG_ACCEL_CONFIG);
    value &= 0b11100111;
    value |= (range << 3);

    port.write(MPU6050_REG_ACCEL_CONFIG, value);
}

void MPU6050::setGyroRange( uint8_t scale)
{
    uint8_t value;

    value = port.read(MPU6050_REG_GYRO_CONFIG);
    value &= 0b11100111;
    value |= (scale << 3);

    port.write(MPU6050_REG_GYRO_CONFIG, value);
}


void MPU6050::setDLPF( uint8_t setting)
{
    uint8_t value;

    value = port.read(MPU6050_REG_CONFIG);
    value &= 0b11111000;
    value |= setting;

    port.write(MPU6050_REG_CONFIG, value);
}

void MPU6050::setSleepEnabled(bool state)
{
    port.writeBit(MPU6050_REG_PWR_MGMT_1, 6, state);
}

void MPU6050::setAccelOffsetX( int16_t offset)
{
	uint8_t data[2];

	data[0] = (uint8_t)(offset >> 8);
	data[1] = (uint8_t)offset;

	port.write(MPU6050_REG_ACCEL_XOFFS_H, data, 2);
}


int16_t MPU6050::getAccelOffsetX( void )
{
	port.read(MPU6050_REG_ACCEL_XOFFS_H, buff, 2);
	return (int16_t)((buff[0] << 8) | buff[1]); 
}

void MPU6050::setAccelOffsetY( int16_t offset )
{
	uint8_t data[2];

	data[0] = (uint8_t)(offset >> 8);
	data[1] = (uint8_t)offset;

	port.write(MPU6050_REG_ACCEL_YOFFS_H, data, 2);
}

void MPU6050::setAccelOffsetZ( int16_t offset )
{
	uint8_t data[2];

	data[0] = (uint8_t)(offset >> 8);
	data[1] = (uint8_t)offset;

	port.write(MPU6050_REG_ACCEL_ZOFFS_H, data, 2);
}

void MPU6050::setGyroOffsetX( int16_t offset )
{
	uint8_t data[2];

	data[0] = (uint8_t)(offset >> 8);
	data[1] = (uint8_t)offset;

	port.write(MPU6050_REG_GYRO_XOFFS_H, data, 2);
}

int16_t MPU6050::getGyroOffsetX( void )
{
	port.read(MPU6050_REG_GYRO_XOFFS_H, buff, 2);
	return (int16_t)((buff[0] << 8) | buff[1]); 
}

void MPU6050::setGyroOffsetY( int16_t offset )
{
	uint8_t data[2];

	data[0] = (uint8_t)(offset >> 8);
	data[1] = (uint8_t)offset;

	port.write(MPU6050_REG_GYRO_YOFFS_H, data, 2);
}


void MPU6050::setGyroOffsetZ( int16_t offset )
{
	uint8_t data[2];

	data[0] = (uint8_t)(offset >> 8);
	data[1] = (uint8_t)offset;

	port.write(MPU6050_REG_GYRO_ZOFFS_H, data, 2);
}



void MPU6050::getData(){

  // Read the next 14 registers starting from MPU6050_REG_ACCEL
  port.read(MPU6050_REG_ACCEL, buff, 14);

  rawAccel.x = (int16_t)((buff[0] << 8) | buff[1]); // X
  rawAccel.y = (int16_t)((buff[2] << 8) | buff[3]); // Y
  rawAccel.z = (int16_t)((buff[4] << 8) | buff[5]); // Z

  // tempRaw = (int16_t)((buff[6] << 8) | buff[7]); // Temperatur
 
  rawGyro.x = (int16_t)((buff[8] << 8) | buff[9]);    // X
  rawGyro.y = (int16_t)((buff[10] << 8) | buff[11]);  // Y
  rawGyro.z = (int16_t)((buff[12] << 8) | buff[13]);; // Z

}

void MPU6050::clearOffsets( void ){
	this->setAccelOffsetX( 0 );
	this->setAccelOffsetY( 0 );
	this->setAccelOffsetZ( 0 );
	this->setGyroOffsetX( 0 );
	this->setGyroOffsetY( 0 );
	this->setGyroOffsetZ( 0 );
}


void MPU6050::calculateMeans( void ){

	// Serial.println("Measuring");

	uint8_t samples = 200;
	int32_t buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;

	this->getData(); // Discard the first measurement 

	for(uint8_t i = 0; i < samples; i++)
	{
		this->getData();

		buff_ax += rawAccel.x;
		buff_ay += rawAccel.y;
		buff_az += rawAccel.z;
		buff_gx += rawGyro.x;
		buff_gy += rawGyro.y;
		buff_gz += rawGyro.z;

		delay(1);
	}

	mean_ax = buff_ax / samples;
	mean_ay = buff_ay / samples;
	mean_az = buff_az / samples;
	mean_gx = buff_gx / samples;
	mean_gy = buff_gy / samples;
	mean_gz = buff_gz / samples;
}


void MPU6050::calibrate( void ){

	int accel_deadzone 	= 8;
	int gyro_deadzone 	= 1;

	this->clearOffsets();	

	this->calculateMeans();

	ax_offset = -mean_ax / 8;
	ay_offset = -mean_ay / 8;
	az_offset = (16384 - mean_az) / 8;

	gx_offset = -mean_gx/4;
	gy_offset = -mean_gy/4;
	gz_offset = -mean_gz/4;


	while (1){

		int ready = 0;

		this->setAccelOffsetX( ax_offset );
		this->setAccelOffsetY( ay_offset );
		this->setAccelOffsetZ( az_offset );
		this->setGyroOffsetX( gx_offset );
		this->setGyroOffsetY( gy_offset );
		this->setGyroOffsetZ( gz_offset );

		Serial.println( "ax:" + (String)ax_offset );
		Serial.println( "aY:" + (String)ay_offset );
		Serial.println( "aZ:" + (String)az_offset );

		Serial.println( "gX:" + (String)gx_offset );
		Serial.println( "gY:" + (String)gy_offset );
		Serial.println( "gZ:" + (String)gz_offset );

		this->calculateMeans();

		if (abs(mean_ax) <= accel_deadzone) ready++;
			else ax_offset -= mean_ax / accel_deadzone;
		
		if (abs(mean_ay) <= accel_deadzone) ready++;
			else ay_offset -= mean_ay / accel_deadzone;

		if (abs(16384 - mean_az) <= accel_deadzone) ready++;
			else az_offset += (16384 - mean_az) / accel_deadzone;

		if (abs(mean_gx) <= gyro_deadzone) ready++;
			else gx_offset -= mean_gx / (gyro_deadzone + 1);
		
		if (abs(mean_gy) <= gyro_deadzone) ready++;
			else gy_offset -= mean_gy / (gyro_deadzone + 1);

		if (abs(mean_gz) <= gyro_deadzone) ready++;
			else gz_offset -= mean_gz / (gyro_deadzone + 1);
	
		if (ready == 6) break;
	}
 
  offsets.ax = ax_offset;
  offsets.ay = ay_offset;
  offsets.az = az_offset;
  offsets.gx = gx_offset;
  offsets.gy = gy_offset;
  offsets.gz = gz_offset;
 
}

void MPU6050::loadOffsets( void ){
  this->setAccelOffsetX( offsets.ax );
  this->setAccelOffsetY( offsets.ay );
  this->setAccelOffsetZ( offsets.az );
  this->setGyroOffsetX( offsets.gx );
  this->setGyroOffsetY( offsets.gy );
  this->setGyroOffsetZ( offsets.gz );
}
