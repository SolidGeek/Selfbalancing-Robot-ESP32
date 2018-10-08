#include "I2C.h"
// #include "Kalman.h"

I2C MCP6050(0x68);

// Kalman kalmanX; // Create the Kalman instances
// Kalman kalmanY;

// double roll, pitch;

uint8_t buff[6]; // 14 bytes of data buffer

uint16_t accel[3]; 
uint16_t gyro[3]; 
// uint32_t timer;

void setup(){
  
  Serial.begin(9600);

  /*buff[0] = 0x07; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  buff[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  buff[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  buff[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g */

  // Set the clock
  MCP6050.write(0x6B, 0x01); // PLL with X axis gyroscope reference and disable sleep

  // Set fullscale gyro range
  MCP6050.write(0x1B, 0x00);

  // Set Full Scale Range to 2000 dps (gyro)
  MCP6050.write(0x1B, 0b11 << 3);

  // Set Full scale range to 2g+- (accel)
  MCP6050.write(0x1C, 0b00 << 3);

  delay(100); // Wait for sensor to stabilize

  // Read the initial sensor values
  readData();

  /*kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);

  timer = micros();*/
  
}

uint8_t response;

// double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

void loop(){

  readData();

  /*double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  double gyroXrate = gyro[0] / 131.0; // Convert to deg/s
  double gyroYrate = gyro[1] / 131.0; // Convert to deg/s

  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    
    kalmanX.setAngle(roll);
    kalAngleX = roll;

  } else{
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
  }

  if (abs(kalAngleX) > 90){
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  }
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);

  Serial.println("Roll: " + String(roll));
  Serial.println("Pitch: " + String(pitch));

  delay(2);*/
}


void readData(){

  response = MCP6050.read(0x3B, buff, 6);

  // Serial.println(buff[0]);

  accel[0] = (int16_t)((buff[0] << 8) | buff[1]); // X
  accel[1] = (int16_t)((buff[2] << 8) | buff[3]); // Y
  accel[2] = (int16_t)((buff[4] << 8) | buff[5]); // Z

  // tempRaw = (int16_t)((buff[6] << 8) | buff[7]); // Temperatur
  /*
  gyro[0] = (int16_t)((buff[8] << 8) | buff[9]);    // X
  gyro[1] = (int16_t)((buff[10] << 8) | buff[11]);  // Y
  gyro[2] = (int16_t)((buff[12] << 8) | buff[13]);; // Z*/

  Serial.println(accel[0]);
  Serial.println(accel[1]);
  Serial.println(accel[2]);
  
  // roll = atan2(accel[1], accel[2]) * RAD_TO_DEG;
  // pitch = atan(-accel[0] / sqrt(accel[1] * accel[1] + accel[2] * accel[2])) * RAD_TO_DEG;
  
}

