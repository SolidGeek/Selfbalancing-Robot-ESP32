#include <Arduino.h>

class Controller{

public:

	Controller( void );
	Controller( float Kp, float Ki, float Kd);

	float Kp = 0.0;
	float Ki = 0.0;
	float Kd = 0.0;

	float error = 0.0;
	float output = 0.0;
	float integral = 0.0;
	float derivative = 0.0;
	float previousError = 0.0;

	int32_t timer = 0;

	void findError( float setpoint, float value );
	float getOutput( void );
	void setConstants( float p, float i, float d );

};