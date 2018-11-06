#include <WiFi.h>
#include "SPIFFS.h"
#include "ESPAsyncWebServer.h"
#include "MPU6050.h"
#include <math.h> 

MPU6050 MPU;

AsyncWebServer server(80);

/* Login oplysninger til AP */
const char *ssid = "BeerBot v1.0";
const char *password = "12345679";

float sensitivity = 16.4;
double compAngle;
double gyroAngle;
uint32_t timer;
double alfa = 0.96;


String processor(const String& var)
{ 
  if(var == "ANGLE"){
    return String(compAngle);
  }
 
  return String();
}

void setup(){
  
  Serial.begin(9600);

  if(!SPIFFS.begin()){
     Serial.println("An Error has occurred while mounting SPIFFS");
     return;
  }

  WiFi.disconnect(true);
  WiFi.mode(WIFI_AP);

  WiFi.softAP(ssid, password);
 
  Serial.print( "IP address: " );
  Serial.println( WiFi.softAPIP() );

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", "text/html");
  });

  server.on("/data.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/data.html", "text/html", false, processor);
  });

  server.on("/virtualjoystick.js", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/virtualjoystick.js", "text/javascript");
  });
 
  server.begin();

  Serial.println("Selfbalancing robot");
  
  if(MPU.begin()){
    Serial.println("Ready");  
  
    MPU.calibrate();
  
    Serial.println( "Calibration done, offsets found:" );
    
    Serial.println( "ax:" + (String)MPU.ax_offset );
    Serial.println( "aY:" + (String)MPU.ay_offset );
    Serial.println( "aZ:" + (String)MPU.az_offset );
    
    Serial.println( "gX:" + (String)MPU.gx_offset );
    Serial.println( "gY:" + (String)MPU.gy_offset );
    Serial.println( "gZ:" + (String)MPU.gz_offset );
      
  }else{
    Serial.println("Couldn't reach MPU");
  }

  
}

void loop(){

  // Get data from MPU6050 
  MPU.getData();

  // Calculate delta time
  double dt = (double)(micros() - timer) / 1000000.0; 
  timer = micros();

  double accelAngle = accelAngleAtan( MPU.rawAccel.y, MPU.rawAccel.z );

  double gyroRate = ( ( MPU.rawGyro.x / sensitivity ));

  gyroAngle += gyroRate * dt;

  compAngle = alfa * (compAngle + gyroRate * dt) + (1.0 - alfa) * accelAngle;

  /*Serial.print(accelAngle);
  Serial.print(',');
  Serial.print(gyroAngle);
  Serial.print(',');
  Serial.println(compAngle);*/
  
  delayMicroseconds(2);

}

double accelAngleAtan( int ay, int az ){

  if(az <= 0 && ay >= 0){
    return 90.0;  
  }

  if(az <= 0 && ay <= 0){
    return -90.0;  
  }
  
  double angle = atan( double((float)ay/(float)az) );

  return angle * 57.2957795;
  
}


