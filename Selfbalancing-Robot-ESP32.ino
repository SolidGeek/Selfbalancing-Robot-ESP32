#include <WiFi.h>
#include "SPIFFS.h"
#include "ESPAsyncWebServer.h"
#include "MPU6050.h"
#include <math.h> 

MPU6050 MPU;

AsyncWebServer server(80);
AsyncWebSocket ws("/data");

/* Login oplysninger til AP */
const char *ssid = "BeerBot v1.0";
const char *password = "12345679";

float sensitivity = 16.4;
double compAngle;
double gyroAngle;
uint32_t timer;
double alfa = 0.96;

void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len){
  if(type == WS_EVT_CONNECT){

    Serial.println("Client connected");
    
  }else if(type == WS_EVT_DATA){
 
 
    for(int i=0; i < len; i++) {
          Serial.print((char) data[i]);
    }

    Serial.println();
 
    client->text( String(compAngle) );
  }
}

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

  // Tell the EPS how to handle the ajax call
  server.on("/data.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/data.html", "text/html", false, processor);
  });

  // Tell the ESP how to include the joystick script
  server.on("/nipplejs.min.js", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/nipplejs.min.js", "text/javascript");
  });

  // Tell the ESP32 how to include style
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/style.css", "text/css");
  });

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
 
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


