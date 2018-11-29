/* Include dependencies */
#include <WiFi.h>
#include <EEPROM.h>
#include <HardwareSerial.h>
#include <math.h> 
#include "SPIFFS.h"
#include "ESPAsyncWebServer.h"
#include "MPU6050.h"
#include "Controller.h"

/* Defining objects for Serial communication with left and right motor driver */
HardwareSerial LeftSerial(1);
HardwareSerial RightSerial(2);

/* Initiation of MPU object */
MPU6050 MPU;

/* Initiation of WebServer and WebSocket object */
AsyncWebServer server(80);
AsyncWebSocket websocket("/datastream");

/* Login details for the access point */
const char * ssid = "RoboTender v1";
const char * password = "12345679";

/* Variables for angle estimation */
double currentAngle = 0;        
double accelAngle = 0.0;
double gyroAngle = 0.0;
const double sensitivity = 16.4;
const double alfa = 0.98;
uint32_t timer = 0;
bool calibrate = false;

/* Buffers to save left and right speed for UART com */
char leftSpeed[10];
char rightSpeed[10];
bool parked = false;

/* Websocket communication timeouts */
unsigned long lastSpeedSet = 0;
unsigned long lastPacket = 0;
bool clientConnected = false;

/* Variables for controller implementation */
float leftVelocity = 0.0, rightVelocity = 0.0;
float angleSetpoint = 0.0;
float velocitySetpoint = 0.0;

float angleSetpointOffset = 0.0;

/* Setup of PID controllers to hold all necessary variables */
Controller inner( -3.5, -1.0, -0.5 ); // Kp = 25, Ki 35, Kd 0.25
Controller outer( 0.0, 0.0, 0.0 ); //  Kp, Ki, Kd = 0.2, 1.0 , 0.0


const float R = 0.07; // Wheel radius 7cm
const float B = 0.19; // Distance between wheels 19cm
const float CUNICYCLE = B / ( 2.0 * R );

/* Function for an incoming websocket connection */
void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len){
  if( type == WS_EVT_CONNECT )
  {
    clientConnected = true;
  }
  else if( type == WS_EVT_DISCONNECT )
  {
    clientConnected = false;  
  }
  else if( type == WS_EVT_DATA )
  {
    // Begin parse data received
    lastPacket = millis();

    char * charArray = (char*)data;
    char * ptr;
    bool newData = false;
    
    if( charArray[0] == '<' ){
    
      charArray++; // Remove <
      
      ptr = strchr(charArray, '>'); // Find end of data stream and replace with null
      if (ptr != NULL) {
          *ptr = '\0';
          newData = true;
      }
    }

    if(newData == true){
      char * index; // this is used by strtok() as an index
      char tempBuffer[10];

      // Translational speed
      index = strtok(charArray,":"); 
      strcpy(tempBuffer, index);

      // Rotational speed
      index = strtok(NULL, ":"); 
      strcpy(tempBuffer, index);

      // Kp for angle controller
      index = strtok(NULL, ":"); 
      strcpy(tempBuffer, index);
      inner.Kp = atof(tempBuffer);

      // Ki for angle controller
      index = strtok(NULL, ":"); 
      strcpy(tempBuffer, index);
      inner.Ki = atof(tempBuffer);

      // Kd for angle controller
      index = strtok(NULL, ":"); 
      strcpy(tempBuffer, index);
      inner.Kd = atof(tempBuffer);

      // Kp for translational controller
      index = strtok(NULL, ":"); 
      strcpy(tempBuffer, index);
      outer.Kp = atof(tempBuffer);

      // Ki for translational controller
      index = strtok(NULL, ":"); 
      strcpy(tempBuffer, index);
      outer.Ki = atof(tempBuffer);

      // Kd for translational controller
      index = strtok(NULL, ":"); 
      strcpy(tempBuffer, index);
      outer.Kd = atof(tempBuffer);

    }

    // Write the current angle back through the webSocket
    client->text( "{\"angle\":" + String(currentAngle) + ",\"leftSpeed\":" + String(leftVelocity) + ",\"rightSpeed\":" + String(rightVelocity) + ",\"angleSetpoint\":" + String(angleSetpoint) + ",\"angleError\":" + String(inner.error) + ",\"angleIntegral\":" + String(inner.integral) + ",\"transSetpoint\":" + String(velocitySetpoint) + ",\"transError\":" + String(outer.error) + ",\"transIntegral\":" + String(outer.integral) + "}" );
  }
}

void setup(){

  // Prepare Serial communication for debugging
  Serial.begin(9600);

  // Prepare EEPROM for retrieving and saving offsets calculated for IMU (inertial measurement unit)
  if (!EEPROM.begin(4096)){
    Serial.println("Failed to initialise EEPROM");
  }

  // Prepare file system to host webserver files
  if(!SPIFFS.begin()){
     Serial.println("Failed to initialise SPIFFS");
     return;
  }

  pinMode(2, OUTPUT);

  // Prepare Serial communication for sending speeds
  RightSerial.begin(115200, SERIAL_8N1, 16, 17);   // RX: 16, TX: 17
  LeftSerial.begin(115200, SERIAL_8N1, 32, 33);  // RX: 32, TX: 33

  
  // Initiate access point for WLAN control
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
  server.on("/smoothie.min.js", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/smoothie.min.js", "text/javascript");
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

uint32_t innerTimer = 0;
uint32_t outerTimer = 0;

void loop()
{

  digitalWrite(2, clientConnected);
  
  if(micros() - innerTimer >= 20000)    // 50 Hz
  {
    estimateAngle();

    // Calculate the motor speeds from the current angle and the desired angle (setpoint)
    angleController( angleSetpoint + angleSetpointOffset , 0.0, currentAngle);
    
    // Set the calculated motor speeds
    setSpeeds();

    innerTimer = micros();
  }

  if(micros() - outerTimer >= 50000) // 20 Hz
  {
    // Calculate angle setpoint needed to obtain desired velocity
    translationalController( 0.0 , leftVelocity, rightVelocity ); // Set setPoint to 0
    
  }
}

void setSpeeds(){

  snprintf(rightSpeed, 10, "%.2f", rightVelocity);
  snprintf(leftSpeed, 10, "%.2f", leftVelocity);

  // Send the calculated speeds as a char array over UART, and end with a newline (stop character)
  RightSerial.print(rightSpeed);
  RightSerial.write('\n');

  LeftSerial.print(leftSpeed);
  LeftSerial.write('\n');
  
}

/*
const float velocityKp = 0.30;
const float velocityKi = 0.05;
const float velocityKd = 0.00;*/

/* Outer controller, controlling the angle setpoint from the reference velocity */
void translationalController(  float setpoint, float leftVel, float rightVel ){

  // Calculate the current translational velocity from RPM to m/s
  float velocity = R * ((leftVel + rightVel)/2.0) * 0.1047198;

  // Calculate error, integral and derivative
  outer.findError( setpoint, velocity );

  // Limit the integral to +/- 0.1 m/s.
  if(outer.integral > 0.2)
    outer.integral = 0.2;  
  else if ( outer.integral < -0.2)
    outer.integral = -0.2; 
  
  // Calculate the nessecary angle setpoint to eliminate any error in translational speed 
  float output = outer.getOutput(); 

  // Limit the output angle to +/- 5 degress

  if(output > 5.0){
    angleSetpoint = 5.0;  
  }else if(output < -5.0){
    angleSetpoint = -5.0;
  }else{
    angleSetpoint = output;
  }

  /*
  Serial.print(velocity);
  Serial.print('\t');
  Serial.print(outer.error);
  Serial.print('\t');
  Serial.print(outer.integral);
  Serial.print('\t');
  Serial.println(angleSetpoint);*/
}

/* Outer controller, controlling the angle setpoint from the reference velocity */
void angleController( float setpoint, float rotation, float angle){

  // Calculate error, integral and derivative
  inner.findError( setpoint, angle );

  // Limit the integral to +/- 2 degree.
  if(inner.integral > 1.0)
    inner.integral = 1.0;  
  else if ( inner.integral < -1.0)
    inner.integral = -1.0;  

  // Calculate the nessecary motor speed to eliminate any error (go to setpoint)
  float output = inner.getOutput();

  // Add the rotational speed 
  float rotVelocity = (rotation * CUNICYCLE) * 30/(R * M_PI);

  // Add the output to the current velocity, to make it a change in velocity from the current velocity
  leftVelocity = (leftVelocity + output) + rotVelocity;
  rightVelocity = (rightVelocity + output) - rotVelocity;

  // Limit the velocity to +/- 200 RPM
  if(leftVelocity > 200.0)
    leftVelocity = 200.0;  
  else if(leftVelocity < -200.0)
    leftVelocity = -200.0;
  
  if(rightVelocity > 200.0)
    rightVelocity = 200.0;  
  else if(rightVelocity < -200.0)
    rightVelocity = -200.0;
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
