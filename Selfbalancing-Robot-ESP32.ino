/* Include dependencies */
#include <WiFi.h>
#include <EEPROM.h>
#include <Servo.h>
#include <HardwareSerial.h>
#include <math.h> 
#include "SPIFFS.h"
#include "ESPAsyncWebServer.h"
#include "MPU6050.h"
#include "Controller.h"

/* Parking */
Servo parkingLeg;
const int servoPin = 13;
uint8_t parkAngle = 98;
uint8_t noparkAngle = 6;

/* Defining objects for Serial communication with left and right motor driver */
HardwareSerial LeftSerial(1);
HardwareSerial RightSerial(2);

/* Initiation of MPU object */
MPU6050 MPU;

/* Very secret login details for the access point */
const char * ssid = "RoboTender v1";
const char * password = "12345679";

/* Initiation of WebServer and WebSocket object */
AsyncWebServer server(80);
AsyncWebSocket websocket("/datastream");


/* Variables for angle estimation */
const double sensitivity = 131.0;
const double alpha = 0.996;
double currentAngle = 0;        
double accelAngle = 0.0;
double gyroAngle = 0.0;
double gyroRate = 0.0;
uint32_t timer = 0;
bool calibrate = false;

float centerOfmassOffset = 1.0; // Angle offset to balance the robot


/* Control variables */
bool parked = false;
uint8_t statusLED = 2;
uint32_t innerTimer = 0;
uint32_t outerTimer = 0;
uint32_t sampleTimer = 0;


/* Websocket communication */
uint32_t lastPacket = 0;
bool clientConnected = false;


/* Variables for controller implementation */
float leftVelocity  = 0.0, rightVelocity = 0.0;
float angleSetpoint = 0.0;	// Reference angle (setpoint) for inner controller
float transVelocity = 0.0; 	// Desired translational velocity (setpoint for outer controller)
float rotVelocity   = 0.0;	  // Desired rotational speed
const float R = 0.07;                     // Wheel radius 7cm
const float B = 0.19;                     // Distance between wheels 19cm
const float CUNICYCLE = B / ( 2.0 * R );  // Wheel turns to achieve one rotation

struct Config{
	float Kp1, Ki1, Kd1, Kp2, Ki2, Kd2;
} config; 

uint8_t offsetAddress = 0;
uint8_t configAddress = 24; // 12 bytes after MPU offsets

/* Setup of PID controllers to hold all necessary variables */
Controller inner;
Controller outer;

float maxWeight = 0.0;
unsigned long weightTimer = 0;


/* Function to handle incoming websocket connection */
void onWsEvent( AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len ){
	if( type == WS_EVT_CONNECT ){
		clientConnected = true;
	}
	else if( type == WS_EVT_DISCONNECT ){
		clientConnected = false;  
	}
	else if( type == WS_EVT_DATA ){

		/* New data has been received on the WebSocket, lets process it */
		bool newData = false;
		
		// Converting the input uint8_t array to char array for compability with string-functions
		char * charArray = (char*)data;
	
		// Lets extract the data string from the char array. Data structure: <"command":"data">
		if( charArray[0] == '<' ){
		
			charArray++; // Remove <
			
			char * ptr = strchr(charArray, '>'); // Find end of data stream and replace with null
			if (ptr != NULL) {
				*ptr = '\0'; // Insert null to indicate end of string
				newData = true;
			}
		}

		// Extract the command string
		if(newData == true){

      lastPacket = millis();

			char command[10];
			char tempBuffer[10];
			char * index;

			index = strtok(charArray,":"); 
			strcpy(command, index);

			if( strcmp(command, "control") == 0 ){

				index = strtok(NULL,":"); 
				strcpy(tempBuffer, index);
				transVelocity = atof(tempBuffer);

				// Rotational speed
				index = strtok(NULL, ":"); 
				strcpy(tempBuffer, index);
				rotVelocity = atof(tempBuffer);

				// Write back the realtime data through the webSocket
				client->text( 
				"{\"type\": \"data\",\"angle\":" 	+ String(currentAngle)  + ",\"leftSpeed\":"   + String(leftVelocity)  + ",\"rightSpeed\":"    + String(rightVelocity) + 
				",\"angleSetpoint\":" 	        + String(angleSetpoint) + ",\"angleError\":"  + String(inner.error)   + ",\"angleIntegral\":" + String(inner.integral) + ",\"angleDerivative\":" + String(inner.derivative) +
				",\"transSetpoint\":" 	        + String(transVelocity) + ",\"transError\":"  + String(outer.error)   + ",\"transIntegral\":" + String(outer.integral) + ",\"transDerivative\":" + String(outer.derivative) + "}"
				);

			}
			else if( strcmp(command, "config") == 0 ){

				/* Variables for PID tuning */
 
				// Kp for angle controller
				index = strtok(NULL, ":"); 
				strcpy(tempBuffer, index);
				config.Kp1 = atof(tempBuffer);

				// Ki for angle controller
				index = strtok(NULL, ":"); 
				strcpy(tempBuffer, index);
				config.Ki1 = atof(tempBuffer);

				// Kd for angle controller
				index = strtok(NULL, ":"); 
				strcpy(tempBuffer, index);
				config.Kd1 = atof(tempBuffer);

				// Kp for translational controller
				index = strtok(NULL, ":"); 
				strcpy(tempBuffer, index);
				config.Kp2 = atof(tempBuffer);

				// Ki for translational controller
				index = strtok(NULL, ":"); 
				strcpy(tempBuffer, index);
				config.Ki2 = atof(tempBuffer);

				// Kd for translational controller
				index = strtok(NULL, ":"); 
				strcpy(tempBuffer, index);
				config.Kd2 = atof(tempBuffer);

				saveConfig();

				inner.setConstants(config.Kp1, config.Ki1, config.Kd1);
				outer.setConstants(config.Kp2, config.Ki2, config.Kd2);
			}

			else if( strcmp(command, "read") == 0 ){
				// No data, the command is enough - return the current config
				Serial.println("App reading config");
				
				client->text( 
				"{\"type\": \"config\",\"Kp1\":" + String(inner.Kp) + ",\"Ki1\":" + String(inner.Ki) + ",\"Kd1\":" + String(inner.Kd) +
				",\"Kp2\":" + String(outer.Kp) + ",\"Ki2\":" + String(outer.Ki) + ",\"Kd2\":" + String(outer.Kd) + "}"
				);
			}
			
			else if( strcmp(command, "calibrate") == 0 ){

				// No data, the command is enough - lets calibrate the IMU!
				if(MPU.isReady()){
					Serial.println( "Calibrating" );
			
					MPU.calibrate();
					EEPROM.put( offsetAddress, MPU.offsets );
					EEPROM.commit(); 
					
					Serial.println( "Calibration done" );
				}else{
					Serial.println( "IMU isn't initialised" );
				}

			}
			
			else if( strcmp(command, "park") == 0 ){
				// No data, the command is enough - lets park!
        if(parked == true){
          parkingLeg.write(noparkAngle);
        }else{
          parkingLeg.write(parkAngle);
        }

        parked = !parked;
        
			}
		}
	}
}

bool control = false;

void setup(){
	// Prepare Serial communication for debugging
	Serial.begin(115200);

	// Prepare EEPROM for retrieving and saving offsets calculated for IMU (inertial measurement unit)
	if (!EEPROM.begin(4096)){
		Serial.println("Failed to initialise EEPROM");
	}
 
	// Prepare file system to host webserver files
	if(!SPIFFS.begin()){
		Serial.println("Failed to initialise SPIFFS");
	}
 
  // Prepare Serial communication for sending speeds
  RightSerial.begin(115200, SERIAL_8N1, 16, 17);   // RX: 16, TX: 17
  LeftSerial.begin(115200, SERIAL_8N1, 32, 33);  // RX: 32, TX: 33
  
  // Set servo pin 
  parkingLeg.attach(servoPin);

	// Prepare pin built-in LED to be used to signal a client connected
	pinMode(statusLED, OUTPUT);

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

  // Load controller configuration from EEPROM
  loadConfig();

	// Initiate the MPU connection
	if(MPU.begin()){

		// Get offsets from EEPROM
		EEPROM.get( offsetAddress, MPU.offsets );
		MPU.loadOffsets();
    
	}else{
		Serial.println("Couldn't reach MPU");
	}

  inner.maxOutput     = 300.0;
  inner.maxIntegral   = 1.5; // 1
  outer.maxOutput     = 15.0;
  outer.maxIntegral   = 0.3; // 0.1
 
}

long timeout = 0;
float deltaTime = 0;

void loop()
{
	// Turn on the status LED if a client i connected
	digitalWrite(statusLED, clientConnected);
 
	if(micros() - innerTimer >= 4000)    // 250 Hz
  {

    // Calculate delta time in seconds (dividing by 10e6)
    deltaTime = (float)(micros() - timer) / 1000000.0; 
    timer = micros();

    // Calculate angle using complimentary filter
    estimateAngle( deltaTime );

    // Balance the robot using PID cascade
	  controller();

    innerTimer = micros();
  
  }

  if( abs(accelAngle) < 5.0 && control == false && parked == false){

    // Initialize the current angle to the current accelAngle
    currentAngle = accelAngle;
    control = true;
    inner.reset();
    outer.reset();

    parkingLeg.write(noparkAngle);

  }

  if( abs(currentAngle) > 30.0){
    control = false;  
  }

  timeout = millis() - lastPacket;

	// Check for lost websocket connection / timeout
	if( timeout > 100)
	{
		transVelocity = 0.0;
		rotVelocity = 0.0;
	}
}

void controller(){
  /*if(micros() - sampleTimer >= 2000) // 500 Hz
  {
     estimateAngle();

     sampleTimer = micros();
  }*/
    
    // Calculate the motor speeds from the current angle and the desired angle (setpoint)
    angleController( angleSetpoint + centerOfmassOffset, rotVelocity, currentAngle );

    // Calculate the anglesetpoint from the current and target velocity
    translationalController( transVelocity, leftVelocity, rightVelocity );
      
    if(control == true){
      // Set the calculated motor speeds
      setSpeeds(leftVelocity, rightVelocity, false);
    }else{
      setSpeeds(0.0, 0.0, true);  
    }  

  /*if(micros() - outerTimer >= 50000) // 20 Hz
  {
    // Calculate angle setpoint needed to obtain desired velocity
    
    // calculatePayloadWeight();
    

    outerTimer = micros();
  }*/
}

void setSpeeds( float left, float right, bool freeWheel ){
  
  // Buffers to hold the left- and rightSpeed before sending over UART
  char leftSpeed[10];   
  char rightSpeed[10]; 

	// Convert float to char array for UART communication
	snprintf(rightSpeed, 10, "%.2f", right);
	snprintf(leftSpeed, 10, "%.2f", left);

	// Send the calculated speeds as a char array over UART, and end with a newline (stop character)
	RightSerial.print(rightSpeed);
	RightSerial.write('\n');

	LeftSerial.print(leftSpeed);
	LeftSerial.write('\n');
	
}


/* Outer controller, controlling the angle setpoint from the reference velocity */
void translationalController(  float setpoint, float leftVel, float rightVel ){

	// Calculate the current translational velocity from RPM to m/s
	float velocity = R * ((leftVel + rightVel)/2.0) * 0.1047198;

	// Calculate error, integral and derivative
	outer.compute( setpoint, velocity, deltaTime);
	
	// Calculate the nessecary angle setpoint to eliminate any error in translational speed 
	angleSetpoint = outer.getOutput(); 

}

/* Outer controller, controlling the angle setpoint from the reference velocity */
void angleController( float setpoint, float rotation, float angle){

	// Calculate error, integral and derivative
	inner.compute( setpoint, angle, deltaTime );

	// Calculate the nessecary motor speed to eliminate any error (go to setpoint)
	float output = inner.getOutput();

	// Add the rotational speed 
	float rotVelocity = (rotation * CUNICYCLE) * 30/(R * M_PI);

	// Add the output to the current velocity, to make it a change in velocity from the current velocity
	leftVelocity = output + rotVelocity;
	rightVelocity = output - rotVelocity;

	// Alwasys Limit the velocity to +/- 200 RPM
	leftVelocity = constrain(leftVelocity, -300.0, 300.0);
	rightVelocity = constrain(rightVelocity, -300.0, 300.0);

}


void estimateAngle( float dt ){

	// Get accel and gyro data from MPU6050 
	MPU.getData();

	// Estimate angle from accelerometer values with geometry
	accelAngle = calculateAccelAngle( MPU.rawAccel.x, MPU.rawAccel.z );

	// Estimate angle from the integration of angular velocity (gyro value) over time
	gyroRate = ( MPU.rawGyro.y / sensitivity );
	gyroAngle += gyroRate * dt;
  
	currentAngle = (alpha) * (currentAngle + gyroRate * dt) + (1.0-alpha) * accelAngle;

  // Serial.println(currentAngle);

  /*uint32_t timerNow = micros();
  Serial.print(accelAngle);
  Serial.print(',');
  Serial.print(gyroAngle);
  Serial.print(',');
  Serial.println(currentAngle);
  uint32_t timerAfter = micros() - timerNow;
  
  Serial.println(timerAfter);*/

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

void calculatePayloadWeight(){

  float fsrVoltage = 0.0;
  float tempVoltage = 0.0;
  float weight = 0.0;
  
  for(uint8_t i = 0; i < 10; i++){
    tempVoltage += ((float)analogRead(35) / 4096.0) * 3.30;
    delayMicroseconds(50);
  }

  fsrVoltage = tempVoltage / 10.0;

  if( fsrVoltage == 0.0){
    maxWeight = 0.0;
  }else{
    weight = fsrVoltage * 625.0;

    if( micros() - weightTimer > 10000){
      maxWeight = weight;  
    }
  
    if(weight > maxWeight){
      maxWeight = weight;
      weightTimer = micros();  
    }
  }
}


void loadConfig(){
  EEPROM.get( configAddress, config );
  inner.setConstants(config.Kp1, config.Ki1, config.Kd1);
  outer.setConstants(config.Kp2, config.Ki2, config.Kd2);
}

void saveConfig(){
  EEPROM.put( configAddress, config );
  EEPROM.commit(); 
}

float EMA( float newSample, float oldSample, float alpha ){
  return ((alpha * newSample) + (1.0-alpha) * oldSample);  
}

