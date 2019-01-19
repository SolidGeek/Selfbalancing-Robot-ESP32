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

  stepper.setMaxVelocity( 2000 ); // Steps/s
  stepper.setMaxAcceleration( 500 ); // Steps/s
  stepper.setCurrent(60);
  stepper.setHoldCurrent(0); // Free-wheeling at RPM = 0
  
  stepper.setup();
}

uint32_t testTimer = 0;

void loop() {

  if( Serial1.available() ){

    testTimer = micros();
  
    char c = Serial1.read();

    if( c == '\n' ){
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

    stepper.setRPM( perSpeed  );
    
    newSpeed = false;  
  }
  
}
