#include <HardwareSerial.h>
#include <Arduino.h>
#include "A4988.h"
#include "BasicStepperDriver.h"

//Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
#define RPM 200

// Since microstepping is set externally, make sure this matches the selected mode
// If it doesn't, the motor will move at a different RPM than chosen
// 1=full step, 2=half step etc.
#define MICROSTEPS 1

// All the wires needed for full functionality
#define DIR1 2
#define STEP1 3

#define DIR2 5
#define STEP2 6


#define LIMITSWITCH1 8
#define LIMITSWITCH2 9


// 2-wire basic config, microstepping is hardwired on the driver. Creating Stepper object
class stepperMotor : public BasicStepperDriver {
public:  
  unsigned long stepsTaken; //Keeps track of steps taken from home position on Motor
  byte dirPin;
  byte stepPin;
  byte limitSwitchPin;

  stepperMotor(byte dirPin, byte stepPin, byte limitPin) : BasicStepperDriver(MOTOR_STEPS, dirPin, stepPin){
    this->limitSwitchPin = limitPin;
    pinMode(limitSwitchPin, INPUT_PULLUP); //initialize limit switch pin
  }
};

stepperMotor stepper1(DIR1, STEP1, LIMITSWITCH1);
stepperMotor stepper2(DIR2, STEP2, LIMITSWITCH2);

enum states {
  NONE,
  HOME,
  STEP,
  MOVE,
  AZIMUTH,
  ELEVATION,
  DANGER,
  TEMP,
  POWERDOWN
};

//Global Variables
states state;
 
// the setup function runs once when you press reset or power the board
void setup() {
  // initialize serial communication:
  Serial.begin(9600);
  state = NONE;

  //initialize stepper motor
  stepper1.begin(RPM, MICROSTEPS);  
  stepper2.begin(RPM, MICROSTEPS); 
  
  
  Serial.println("Controller is Ready");
  Serial.println("Ensure that line ending is set to both Newline and Carriage Return");//A single command is assumed to end when those characters are read.
}




// the loop function runs over and over again forever
void loop(){
while (Serial.available()>0){
    processData(Serial.read());
  };


  // energize coils - the motor will hold position
  //stepper1.enable();
  //stepper2.enable();

  //Homing();// To Author: Check if steps are being tracked..may need to use a pointer to access the class in memory
  
  //Moving motor one full revolution using the degree notation
  stepper1.rotate(180);
  stepper2.rotate(-180);

  Serial.println("Moving");
  
  
  delay(5*1000);
}



void Homing(long timeout=15){ // Homing position 0 Elevation and 0 Azimuth
  bool Home = false;
  float Time0, TimeErr;
  
 
  Time0 = millis(); // Get time when homing Routine was launched
  TimeErr = timeout*1000; //Time in secs until homing function returns error because limit switch was not found

  while (!Home){
    if (millis()-Time0 >= TimeErr){
      Serial.println("Error 404: Home Not Found...I'm Lost, Help");
      break;
      }

    if ((digitalRead(stepper1.limitSwitchPin) == LOW) && digitalRead(stepper2.limitSwitchPin) == LOW){ //Assuming Normally Open limit switch, when the pin is low the switch is closed and the pin will read low. When it reads low, the homing position is achieved
      Home = true;
      Serial.println("Home Found");
      //For future changes. Adjust homing such that at 0 zero steps taken for each motor represents 0 elevations and 0 azmituth (to represtent by marker on heliostat)
      stepper1.stepsTaken = 0;
      stepper2.stepsTaken = 0;
      break;
    } 
    
   //Move one step until limit switch is reached
   stepper1.move(1);
   stepper2.move(1);
  }  
}


void controllerStep(stepperMotor *stepper, long steps){
  stepper->move(steps);
  stepper->stepsTaken += (steps);
}

void controllerRotate(stepperMotor *stepper, float deg){
  long steps = stepper->calcStepsForRotation(deg);
  controllerStep(stepper,steps);
}

void azimuth(float az){
  long stepNo = stepper1.calcStepsForRotation(az) - (stepper1.stepsTaken)%360;
  controllerStep(&stepper1,stepNo);
}

void elevation(float el){
  long stepNo = stepper2.calcStepsForRotation(el) - (stepper2.stepsTaken)%360;
  controllerStep(&stepper2,stepNo);
}

void danger(){
Serial.println("Danger: Moving to safe position"); 
elevation(90);
azimuth(0);//Honestly probably not necessary since the mirror is flat....but it's here if needed.

};

void temperature(){
  //Need to play around with esp32 board 1st.
  Serial.println("Temperature protocol not implemented"); 
};

void powerDown(){
  //Need clarification on this one...do want to power down and possibly damage it or something else
  Serial.println("Power down protocol not implemented");   
 };

void processData(byte data){
  switch(data){
    case 'H':
      state = HOME;
      break;
    case 'S':
      state = STEP;
      break;
    case 'M':
      state = MOVE;
      break;
    case 'A':
      state = AZIMUTH;
      break;
    case 'E':
      state = ELEVATION;
      break;
    case 'D':
      state = DANGER;
      break;
    case 'T':
      state = TEMP;
      break;
    case 'P':
      state = POWERDOWN;
      break;
    default:
      state = NONE;   
      error();      
      break;
  };
  if (state != NONE){
    processState();
  }
  
};

void error(){ //Flushes Serial Buffer and warns operator
  while (Serial.available()>0){Serial.read();}
  Serial.println("Unknown Command");
};

void processState(){
  switch(state){
    case HOME:
      if ((Serial.read() == '\r')&&(Serial.read() == '\n')) Homing();
      else error();
      break;
    case STEP:
      int axis = Serial.parseInt();
      int stepNo = Serial.parseInt();
      
      if (axis == 0){controllerStep(&stepper1,stepNo);}
      else if (axis == 1){controllerStep(&stepper2,stepNo);}
      else error();
      
      break;
    case MOVE:
      int axis = Serial.parseInt();
      float angle = Serial.parseFloat();
      
      if (axis == 0){controllerRotate(&stepper1,angle);}
      else if (axis == 1){controllerRotate(&stepper2,angle);}
      else error();
      
      break;
    case AZIMUTH: // Remember to ensure that positive convention matches the positive direction for motor rotation
      azimuth(Serial.parseFloat());
      
      break;
    case ELEVATION: // Remember to ensure that positive convention matches the positive direction for motor rotation
      //could turn this into a function
      float el = Serial.parseFloat();
      long stepNo = stepper2.calcStepsForRotation(el) - (stepper2.stepsTaken)%360;

      controllerRotate(&stepper2,angle);

      break;
    case DANGER:
      danger();

      break;
    case 'T':
      temperature();

      break;
    case POWERDOWN:
      powerDown();

      break;
    default:
      Serial.println("This should never occur");

      break;
  };
  while (Serial.available()>0){Serial.read();}
};
