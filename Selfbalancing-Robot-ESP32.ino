#include "MPU6050.h"

MPU6050 MPU;

void setup(){
  
  Serial.begin(9600);
  if(MPU.begin()){
    Serial.println("Ready");  
  }

}

float output = 0;
float angle = 0;

void loop(){

  MPU.getData();

  output += (MPU.accel.x - output) * 0.5;

  angle = (output/16384.0)*90.0;
  

  Serial.println(angle);

  delay(2);

}


