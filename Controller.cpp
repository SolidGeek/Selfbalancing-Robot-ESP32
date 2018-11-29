#include "Controller.h"

Controller::Controller( float Kp, float Ki, float Kd){

	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;

}

void Controller::findError( float setpoint, float value ){

	// Calculate dt in seconds since last loop

	double dt = (double)(micros() - this->timer) / 1000000.0; 
	this->timer = micros();

	// Calculating PID values

	this->error = setpoint - value; 
	this->integral += this->error * dt; 
	this->derivative = (this->error - this->previousError)/dt;

	// Save this error as previousError

	this->previousError = error;

}

float Controller::getOutput(){

	// Calculate output from PID constants and values
	this->output = (this->error * Kp) + (this->integral * Ki) + (this->derivative * Kd);

	return this->output;

}
