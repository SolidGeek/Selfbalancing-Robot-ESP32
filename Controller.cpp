#include "Controller.h"

Controller::Controller( void ){
	
}

Controller::Controller( float Kp, float Ki, float Kd){

	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;

}

void Controller::compute(float setpoint, float input, float deltaTime ){
  
  error = setpoint - input; 

  // Add this cycle's integral to the integral cumulation
  integral += error * deltaTime; 

  // Calculate the slope with the data from the current and last cycle
  derivative = (error - lastError);

  // Prevent the integral cumulation from becoming overwhelmingly huge
  if(integral > maxIntegral) integral = maxIntegral;
  if(integral < -maxIntegral) integral = -maxIntegral;

  // Calculate the controller output based on the data and PID gains
  output = (error * Kp) + (integral * Ki) + (derivative * Kd);

  if(output > maxOutput) output = maxOutput;
  if(output < -maxOutput) output = -maxOutput;


  
  lastError = error;
  
}

void Controller::reset(){
  error = 0.0;
  derivative = 0.0;
  integral = 0.0;
  output = 0.0;
}

float Controller::getOutput(){
	return this->output;
}

void Controller::setConstants( float p, float i, float d ){
	this->Kp = p;
	this->Ki = i;
	this->Kd = d;
}
