#include <uStepperS.h>

uStepperS stepper; 

float perSpeed = 0.0;
char buff[10];
uint8_t index = 0;
unsigned long timeoutTimer = 0;
bool newSpeed = true;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(115200, SERIAL_8N1);

  stepper.setMaxVelocity( 0 ); // Steps/s
  stepper.setMaxAcceleration( 80 ); // Steps/s
  
  stepper.setup();
}

void loop() {
  // put your main code here, to run repeatedly:
  if( Serial1.available() ){

    char c = Serial1.read();

    if( c == '\0' ){
      buff[index++] = '\0';
      index = 0;

      perSpeed = atof( buff );
      
      timeoutTimer = millis();
      newSpeed = true;
      
      memset(buff, 0, sizeof(buff));
      
    }else{
      buff[index++] = c; 
    }
  }

  if(millis() - timeoutTimer > 50){

    perSpeed = 0.0;
    newSpeed = true;
  
  }
  
  if(newSpeed){

    // Serial.println( perSpeed );
    stepper.setRPM( perSpeed  );
    
    newSpeed = false;  
  }
  
}
