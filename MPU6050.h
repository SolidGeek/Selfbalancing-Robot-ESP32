#include <Arduino.h>
#include "I2C.h"

#define MPU6050_REG_ACCEL_XOFFS_H 0x06
#define MPU6050_REG_ACCEL_YOFFS_H 0x08
#define MPU6050_REG_ACCEL_ZOFFS_H 0x0A

#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_GYRO_XOFFS_H 0x13
#define MPU6050_REG_GYRO_YOFFS_H 0x15
#define MPU6050_REG_GYRO_ZOFFS_H 0x17

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
  
  	void setAccelRange( uint8_t range );
  
  	void setGyroRange( uint8_t scale );
  
  	void setClockSource( uint8_t source );

    void setDLPF( uint8_t setting );
  
  	void setSleepEnabled( bool state );

    void setAccelOffsetX( int16_t offset );
    void setAccelOffsetY( int16_t offset );
    void setAccelOffsetZ( int16_t offset );

    int16_t getAccelOffsetX( void );

    void setGyroOffsetX( int16_t offset );
    void setGyroOffsetY( int16_t offset );
    void setGyroOffsetZ( int16_t offset );

    int16_t getGyroOffsetX( void );

    void loadOffsets( void );
  
    void getData();

    void calculateMeans();
    void calibrate();
    void clearOffsets();

    bool isReady();

    Vector rawGyro;
    Vector rawAccel;

    int16_t ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;
    int16_t mean_ax = 0, mean_ay = 0, mean_az = 0, mean_gx = 0, mean_gy = 0, mean_gz = 0;

    struct Offsets{
      int16_t ax = 0;
      int16_t ay = 0;
      int16_t az = 0;
      int16_t gx = 0;
      int16_t gy = 0;
      int16_t gz = 0;
    } offsets;
    
    
  private:
  
  	I2C port;

    int LPF( int newSample, int oldSample, float alpha );
    
    uint8_t buff[14];

    bool ready = false;

    
 
};
