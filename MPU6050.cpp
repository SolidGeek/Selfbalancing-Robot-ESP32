#include "MPU6050.h"

MPU6050::MPU6050(void){}

bool MPU6050::begin(){

  Serial.println();

	port.setAdress( MPU6050_ADRESS );

  delay(100);

  if (port.read(MPU6050_REG_WHO_AM_I) != MPU6050_ADRESS){
    return false;
  }
  
  setSleepEnabled( false ); // Turn of sleep mode before any other registers is configured!!!
  setClockSource( MPU6050_CLOCK_PLL_XGYRO );
  setRange( MPU6050_RANGE_2G );
  setScale( MPU6050_SCALE_2000DPS );

  return true;

}

void MPU6050::setClockSource( uint8_t source )
{
    uint8_t value;
    
    value = port.read(MPU6050_REG_PWR_MGMT_1);
    
    value &= 0b11111000;
    value |= source;
 
    port.write(MPU6050_REG_PWR_MGMT_1, value);
}

void MPU6050::setRange( uint8_t range )
{
    uint8_t value;

    value = port.read(MPU6050_REG_ACCEL_CONFIG);
    value &= 0b11100111;
    value |= (range << 3);

    port.write(MPU6050_REG_ACCEL_CONFIG, value);
}

void MPU6050::setScale( uint8_t scale)
{
    uint8_t value;

    value = port.read(MPU6050_REG_GYRO_CONFIG);
    value &= 0b11100111;
    value |= (scale << 3);

    Serial.print("Write:");
    Serial.println(value, BIN);

    Serial.println(port.write(MPU6050_REG_GYRO_CONFIG, value));
}


void MPU6050::setSleepEnabled(bool state)
{
    port.writeBit(MPU6050_REG_PWR_MGMT_1, 6, state);
}

void MPU6050::getData(){

  // Read the next 14 registers starting from MPU6050_REG_ACCEL
  port.read(MPU6050_REG_ACCEL, buff, 14);

  accel.x = (int16_t)((buff[0] << 8) | buff[1]); // X
  accel.y = (int16_t)((buff[2] << 8) | buff[3]); // Y
  accel.z = (int16_t)((buff[4] << 8) | buff[5]); // Z

  // tempRaw = (int16_t)((buff[6] << 8) | buff[7]); // Temperatur
 
  gyro.x = (int16_t)((buff[8] << 8) | buff[9]);    // X
  gyro.y = (int16_t)((buff[10] << 8) | buff[11]);  // Y
  gyro.z = (int16_t)((buff[12] << 8) | buff[13]);; // Z
  
}
