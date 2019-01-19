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
	float lastError = 0.0;

  float maxOutput= 0.0;

  /* Used for integral windup */
  float maxIntegral = 0.0;

	uint32_t lastTime = 0;

	void compute( float setpoint, float value, float deltaTime);
	float getOutput( void );
	void setConstants( float p, float i, float d );
  void reset( void );

};
