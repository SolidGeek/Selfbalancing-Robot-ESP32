#include <Arduino.h>
#include "I2C.h"

#define MPU6050_REG_ACCEL_XOFFS_H 0x06
#define MPU6050_REG_ACCEL_XOFFS_L 0x07

#define MPU6050_ADRESS            0x68
#define MPU6050_REG_WHO_AM_I      0x75
#define MPU6050_REG_ACCEL         0x3B
#define MPU6050_REG_GYRO_CONFIG 	0x1B // Gyroscope Configuration
#define MPU6050_REG_ACCEL_CONFIG 	0x1C
#define MPU6050_REG_PWR_MGMT_1 		0x6B // Power Management 1

#define MPU6050_CLOCK_PLL_XGYRO	0b001

#define MPU6050_SCALE_2000DPS 	0b11
#define MPU6050_SCALE_1000DPS 	0b10
#define MPU6050_SCALE_500DPS 		0b01
#define MPU6050_SCALE_250DPS 		0b00

#define MPU6050_RANGE_16G 		0b11
#define MPU6050_RANGE_8G 			0b10
#define MPU6050_RANGE_4G 			0b01
#define MPU6050_RANGE_2G 			0b00

class MPU6050
{
  
  struct Vector {
      int16_t x;
      int16_t y;
      int16_t z;
  };
 
  
  public:

    MPU6050(void);
  
  	bool begin();
  
  	void setRange( uint8_t range );
  
  	void setScale( uint8_t scale );
  
  	void setClockSource( uint8_t source );
  
  	void setSleepEnabled( bool state );

    void setAccelOffsetX( int16_t offset );
  
    void getData();
  
    Vector rawGyro;
    Vector rawAccel;

    Vector gyro;
    Vector accel;
  
  private:
  
  	I2C port;

    int EMA( int newSample, int oldSample, float alpha );
  
    uint8_t buff[14];
 
};
