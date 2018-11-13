#include <WiFi.h>
#include "SPIFFS.h"
#include "ESPAsyncWebServer.h"
#include "MPU6050.h"
#include <math.h> 
#include <HardwareSerial.h>

HardwareSerial LeftSerial(1);
HardwareSerial RightSerial(2);

// Initiation of MPU object
MPU6050 MPU;

// Initiation of WebServer and WebSocket object
AsyncWebServer server(80);
AsyncWebSocket websocket("/data");

// Login details for the access point
const char * ssid = "BeerBot v1.0";
const char * password = "12345679";

// Variables for angle estimation
double angle = 0;
double sensitivity = 16.4;
double alfa = 0.96;
double accelAngle = 0, gyroAngle = 0;
uint32_t timer;

char leftSpeed[10];
char rightSpeed[10];
bool parked = false;
unsigned long lastSpeedSet = 0;

unsigned long lastPacket = 0;

// Function for an incoming websocket connection
void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len){
  if(type == WS_EVT_CONNECT){
    Serial.println("Client connected");
  }else if(type == WS_EVT_DATA){

    // Begin parse data received
    lastPacket = millis();

    char * charArray = (char*)data;
      
    char * index; // this is used by strtok() as an index

    index = strtok(charArray,":"); 
    strcpy(leftSpeed, index);

    index = strtok(NULL, ":"); 
    strcpy(rightSpeed, index);
    
    index = strtok(NULL, ":");
    parked = atoi(index); 
    
    client->text( String(angle) );
  }
}

void setup(){
  
  Serial.begin(9600);

  RightSerial.begin(115200, SERIAL_8N1, 16, 17);   // RX: 16, TX: 17
  LeftSerial.begin(115200, SERIAL_8N1, 32, 33);  // RX: 32, TX: 33

  // Initiate file server 
  if(!SPIFFS.begin()){
     Serial.println("An Error has occurred while mounting SPIFFS");
     return;
  }

  // Initiate access point for WiFi control
  WiFi.softAP(ssid, password);

  // Tell the ESP how to handle specific incoming HTTP requests
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", "text/html");
  });
  server.on("/nipplejs.min.js", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/nipplejs.min.js", "text/javascript");
  });
  server.on("/gauge.min.js", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/gauge.min.js", "text/javascript");
  });
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/style.css", "text/css");
  });

  // Tell the WebSocket to execute the function onWsEvent() on each WebSocket event
  websocket.onEvent(onWsEvent);
  server.addHandler(&websocket);
  
  // Begin the WebServer
  server.begin();

  // Initiate the MPU connection
  if(MPU.begin()){

    Serial.println("Ready");  
  
    // Calibrate the accel and gyro
    MPU.calibrate();
    Serial.println( "Calibration done" );
      
  }else{
    Serial.println("Couldn't reach MPU");
  }
}

void loop(){

  estimateAngle();

  // Timeout 
  if(millis() - lastPacket > 100){
    strcpy(leftSpeed, "0");
    strcpy(rightSpeed, "0");
  }

  setSpeeds();

  delayMicroseconds(5);

}

void setSpeeds(){
  if( millis() - lastSpeedSet > 10 ){
    /* Print Speeds by UART to the uSteppers */
    RightSerial.print(rightSpeed);
    RightSerial.write('\0');
  
    LeftSerial.print(leftSpeed);
    LeftSerial.write('\0');

    lastSpeedSet = millis();
  }
}

void estimateAngle(){

  // Get accel and gyro data from MPU6050 
  MPU.getData();

  // Calculate delta time in seconds (dividing by 10e6)
  double dt = (double)(micros() - timer) / 1000000.0; 
  timer = micros();

  // Estimate angle from accelerometer values with geometry
  accelAngle = calculateAccelAngle( MPU.rawAccel.y, MPU.rawAccel.z );

  // Estimate angle from the integration of angular velocity (gyro value) over time
  gyroAngle += ( MPU.rawGyro.x / sensitivity ) * dt;

  angle = alfa * (angle + (MPU.rawGyro.x / sensitivity) * dt) + (1.0 - alfa) * accelAngle;

}

double calculateAccelAngle( int ay, int az ){

  // Limit the angle to 90 and -90 degress
  if(az <= 0 && ay >= 0){
    return 90.0;  
  }

  if(az <= 0 && ay <= 0){
    return -90.0;  
  }
  
  // Calculate argtan and convert to degress by multipliing by 57.296 (180/pi)
  return atan( (double)ay / (double)az ) * 57.2957795;
  
}
