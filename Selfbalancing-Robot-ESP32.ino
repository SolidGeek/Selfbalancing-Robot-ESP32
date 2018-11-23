#include <WiFi.h>
#include <EEPROM.h>
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

/* Login details for the access point */
const char * ssid = "RoboTender v1";
const char * password = "12345679";

/* Variables for angle estimation */
double currentAngle = 0;
double sensitivity = 16.4;
double alfa = 0.98;
double accelAngle = 0, gyroAngle = 0;
uint32_t timer;

/* Buffers to save left and right speed for UART com */
char leftSpeed[10];
char rightSpeed[10];
bool parked = false;

/* Websocket communication timeouts */
unsigned long lastSpeedSet = 0;
unsigned long lastPacket = 0;


/* Variables for controller implementation */
float leftVelocity = 0, rightVelocity = 0;
float angleSetpoint;
float velocitySetpoint;



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
    
    client->text( String(currentAngle) );
  }
}

bool calibrate = false;

void setup(){
  
  Serial.begin(9600);

  if (!EEPROM.begin(4096)){
    Serial.println("Failed to initialise EEPROM");
  }

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

    if( calibrate == true ){
      
      // Calibrate the accel and gyro
      Serial.println( "Calibrating" );
      
      MPU.calibrate();
      EEPROM.put( 0, MPU.offsets );
      EEPROM.commit(); 
      
      Serial.println( "Calibration done" );
      
    }else{

      // Get offsets from EEPROM
      EEPROM.get( 0, MPU.offsets );
      MPU.loadOffsets();
        
    }
    
  }else{
    Serial.println("Couldn't reach MPU");
  }
}

uint32_t D1 = 0, D2 = 0, sample = 0;

void loop(){

  if(micros() - sample >= 2000) // 500 Hz
  {
    estimateAngle();
    sample = micros();

    Serial.println(currentAngle);
  }

  if(micros() - D1 >= 20000)    // 50 Hz
  {
    // Calculate D1 (speed setpoint)
    // angleController(angleSetpoint, currentAngle, 0);
    angleController(0, 0, currentAngle);
    // Set motor speeds
    
    setSpeeds();

    D1 = micros();
  }

  if(micros() - D2 >= 50000) // 50 Hz
  {
    // Calculate D2
    // Calculate angle setpoints
    // translationalController( leftVelocity, rightVelocity, 0); // Set setPoint to 0
  }



  // Timeout 
  /* if(millis() - lastPacket > 100){
    strcpy(leftSpeed, "0");
    strcpy(rightSpeed, "0");
  } */ 

  // setSpeeds();

  // delayMicroseconds(5);

}

void setSpeeds(){
  // if( millis() - lastSpeedSet > 10 ){
    /* Print Speeds by UART to the uSteppers */

    snprintf(rightSpeed, 10, "%.2f", rightVelocity);
    snprintf(leftSpeed, 10, "%.2f", leftVelocity);

    /*Serial.print(rightVelocity);
    Serial.print('\t');
    Serial.println(leftVelocity);*/
    /*Serial.print('\t');
    Serial.print(currentAngle);
    Serial.print('\t');
    Serial.println(angleSetpoint);*/
    
    RightSerial.print(rightSpeed);
    RightSerial.write('\n');
  
    LeftSerial.print(leftSpeed);
    LeftSerial.write('\n');

    // lastSpeedSet = millis();
  // }
}

#define translationKp -0.3f
#define translationKi -0.05f
#define translationKd -0.00f
#define R 0.07f // Wheel radius 7cm
#define B 0.19f // Distance between wheels 19cm
#define CUNICYCLE B/(2.0f*R)  

/* Outer controller, controlling the angle setpoint from the reference velocity */
void translationalController( float leftVel, float rightVel, float setpoint ){

  float error = 0.0, output = 0.0;
  static float totalError = 0.0, errorOld = 0.0;

  float velocity = (R * 2.0) * ((leftVel + rightVel)/4.0); // Total translational velocity in m/s

  error = setpoint - velocity;
  totalError += error;
  errorOld = error;

  if(totalError > 1.0){
    totalError = 1.0;  
  }else if ( totalError < -1.0){
    totalError = -1.0;  
  }

  // PI regulator output
  output = (error * translationKp) + (totalError * translationKi) + (errorOld * translationKd); // Output is the new angle setpoint

  angleSetpoint = output;

  if(output > 20.0){
    angleSetpoint = 20.0;  
  }else if(output < -20.0){
    angleSetpoint = -20.0;
  }
}

#define angleKp -3.5f
#define angleKi -0.0f
#define angleKd -0.0f

/* Outer controller, controlling the angle setpoint from the reference velocity */
void angleController( float setpoint, float rotation, float angle){

  float error = 0.0, output = 0.0;
  static float totalError = 0.0, errorOld = 0.0;
  float temp;

  error = setpoint - angle; // Calculation of error in angle
  totalError += error;
  errorOld = error;

  /*if(totalError > 15.0){
    totalError = 15.0;  
  }else if ( totalError < -15.0){
    totalError = -15.0;  
  }*/

  // PI regulator output
  output = (error * angleKp) + (totalError * angleKi) + (angleKd * errorOld); // Calculation of speeds nessecary to optain desired angle

  temp = (rotation * CUNICYCLE);

  leftVelocity = output + temp;
  rightVelocity = output - temp;

  if(leftVelocity > 200.0){
    leftVelocity = 200.0;  
  }else if(leftVelocity < -200.0){
    leftVelocity = -200.0;
  }

  if(rightVelocity > 200.0){
    rightVelocity = 200.0;  
  }else if(rightVelocity < -200.0){
    rightVelocity = -200.0;
  }
}

void estimateAngle(){

  // Get accel and gyro data from MPU6050 
  MPU.getData();

  // Calculate delta time in seconds (dividing by 10e6)
  double dt = (double)(micros() - timer) / 1000000.0; 
  timer = micros();

  // Estimate angle from accelerometer values with geometry
  accelAngle = calculateAccelAngle( MPU.accel.x, MPU.accel.z );

  // Estimate angle from the integration of angular velocity (gyro value) over time
  gyroAngle += ( MPU.gyro.y / sensitivity ) * dt;

  currentAngle = alfa * (currentAngle + (MPU.gyro.y / sensitivity) * dt) + (1.0 - alfa) * accelAngle;

}

double calculateAccelAngle( int ax, int az ){

  /*
  Serial.print(MPU.rawAccel.x);
  Serial.print(',');
  Serial.print(MPU.rawAccel.y);
  Serial.print(',');
  Serial.println(MPU.rawAccel.z);
  */

  
  // Limit the angle to 90 and -90 degress
  if(az <= 0 && ax <= 0){
    return 90.0;  
  }

  if(az <= 0 && ax >= 0){
    return -90.0;  
  }
  
  // Calculate argtan and convert to degress by multipliing by 57.296 (180/pi)
  return atan( - (double)ax / (double)az ) * 57.2957795;
  
}
