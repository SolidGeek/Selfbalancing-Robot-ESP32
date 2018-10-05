#include "I2C.h"

I2C MCP6050(0x68);

uint8_t returnData[10]; // 10 bytes of data buffer
uint16_t accel[3]; 

void setup(){
  Serial.begin(9600);

  Wire.begin();
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
}

void loop(){

  uint8_t response;

  response = MCP6050.read(0x3B, returnData, 6);

  if(response == 0){

    Serial.println("Accel data:");

    accel[0] = (int16_t)((returnData[0] << 8) | returnData[1]); // X
    accel[1] = (int16_t)((returnData[2] << 8) | returnData[3]); // Y
    accel[2] = (int16_t)((returnData[4] << 8) | returnData[5]); // Z
    
    for(uint8_t x = 0; x < 3; x++){
      Serial.println(accel[x]);  
    }
  }

  delay(1000);
  
}
