//Notes: 
//Major Concern for input validation. Things will probably work when it shouldn't and vice-versa. Do we need to impement user validation? Ans: We don't.
//Delays scattered throughout because the serial.read() can read faster than the rate of the buffer being filled.
//Stepper 1 is the azimuth axis and stepper 2 is the elevation access   
//Limit switches are also used as a safety feature.....motors should stop when limit switches are hit...particularly the elevation motor
/*Motor Info
  Coils for Motor
  A+ Black
  A- Green
  B+ Red
  B- Blue
  Motor Rotation Direction -  With mounting face towards observer
  + = CCW
  - = CW
*/

#include <HardwareSerial.h>
#include <Arduino.h>
#include "A4988.h"
#include "BasicStepperDriver.h"

//Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
#define RPM 120

// Since microstepping is set externally, make sure this matches the selected mode
// If it doesn't, the motor will move at a different RPM than chosen
// 1=full step, 2=half step etc.
#define MICROSTEPS 1

// All the wires needed for full functionality
#define STEP1 14
#define DIR1 32

#define STEP2 15
#define DIR2 33



#define LIMITSWITCH1 13
#define LIMITSWITCH2 12
#define LIMITSWITCH3 999


// 2-wire basic config, microstepping is hardwired on the driver. Creating Stepper object
class stepperMotor : public BasicStepperDriver {
public:  
  long stepsTaken=0; //Keeps track of steps taken from home position on Motor
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

//Prototypes
void controllerStep(stepperMotor *stepper, long steps, float rpm);
void controllerRotate(stepperMotor *stepper, float deg, float rpm);
 
// the setup function runs once when you press reset or power the board
void setup() {
  // initialize serial communication:
  Serial.begin(115200);
  state = NONE;

  //initialize stepper motor
  stepper1.begin(RPM, MICROSTEPS);  
  stepper2.begin(RPM, MICROSTEPS); 
  
  
  Serial.println("Controller is Ready");
  //Old requirement:::Serial.println("Ensure that line ending is set to both Newline and Carriage Return");//A single command is assumed to end when those characters are read.
}




// the loop function runs over and over again forever
void loop(){
  while (Serial.available()>0){
      processData(Serial.read());
  };

//  controllerStep(&stepper1,-100,5);
//  delay(1*1000);
//  controllerStep(&stepper1,50,3);
//  controllerRotate(&stepper2,-3600,200);




  
  Serial.println((String)"Stepper_1 Steps Taken: " + stepper1.stepsTaken);
  Serial.println((String)"Stepper_2 Steps Taken: " + stepper2.stepsTaken);
  delay(0.1*1000);


  // energize coils - the motor will hold position
  //stepper1.enable();
  //stepper2.enable();

  //Homing();// To Author: Check if steps are being tracked..may need to use a pointer to access the class in memory
  
  //Moving motor one full revolution using the degree notation
  //stepper1.rotate(180);
  //stepper2.rotate(-180);

  //stepper1.move(1);
  
}



void Homing(long timeout=20){ // Homing position 0 Elevation and 0 Azimuth
  bool Home1_init = false;bool Home1_final = false;
  bool Home2_init = false;bool Home2_final = false;
  float Time0, TimeErr;

  Serial.println("Homing Protocol Activated");
 
  Time0 = millis(); // Get time when homing Routine was launched
  TimeErr = timeout*1000; //Time in secs until homing function returns error because limit switch was not found
   

    while (!Home1_final){
      while(!Home1_init){
        if (millis()-Time0 >= TimeErr){
          Serial.println("Error 404: Home Not Found...I'm Lost, Help");
          goto bailout;// Skips to end of homing procedure to end it.
        }
//        stepper1.setRPM(30);
//        stepper1.move(10);
          controllerStep(&stepper1,-1,30);delay(10);
        
        if (digitalRead(stepper1.limitSwitchPin) == LOW){ //Assuming Normally Open limit switch, when the pin is low the switch is closed and the pin will read low. When it reads low, the homing position is achieved
          Home1_init = true;
          Serial.println("Stepper 1 Rapid Homing Complete.");  
        }
  
      }//End of rapid homing
      Serial.println("Initializing Stepper 1 Precision Homing");
      delay(1*1000);
//      stepper1.move(-20);
//      stepper1.setRPM(5);
      controllerStep(&stepper1,20,30);delay(500);
      while(digitalRead(stepper1.limitSwitchPin) == HIGH){//Maybe switch bounce or floating voltage is an issue
        controllerStep(&stepper1,-1,5);delay(60);
      }
      delay(50);//Button Debounce
      Serial.println("Home of stepper 1 Found");
      Home1_final=true;
      //For future changes. Adjust homing such that at 0 zero steps taken for each motor represents 0 elevations and 0 azmituth (to represtent by marker on heliostat)
      stepper1.stepsTaken = 0;
    }

   
    Time0 = millis();
    while (!Home2_final){
      while(!Home2_init){
        if (millis()-Time0 >= TimeErr){
          Serial.println("Error 404: Home Not Found...I'm Lost, Help");
          goto bailout;// Skips to end of homing procedure to end it.
        }
//        stepper1.setRPM(30);
//        stepper1.move(10);
          controllerStep(&stepper2,-1,30);delay(10);
        
        if (digitalRead(stepper2.limitSwitchPin) == LOW){ //Assuming Normally Open limit switch, when the pin is low the switch is closed and the pin will read low. When it reads low, the homing position is achieved
          Home2_init = true;
          Serial.println("Stepper 2 Rapid Homing Complete.");  
        }
  
      }//End of rapid homing
      Serial.println("Initializing Stepper 2 Precision Homing");
      delay(1*1000);
//      stepper1.move(-20);
//      stepper1.setRPM(5);
      controllerStep(&stepper2,20,30);delay(500);
      while(digitalRead(stepper2.limitSwitchPin) == HIGH){//Maybe switch bounce or floating voltage is an issue
        controllerStep(&stepper2,-1,5);delay(60);
      }
      delay(50);//Button Debounce
      Serial.println("Home of stepper 1 Found");
      Home2_final=true;
      //For future changes. Adjust homing such that at 0 zero steps taken for each motor represents 0 elevations and 0 azmituth (to represtent by marker on heliostat)
      stepper2.stepsTaken = 0;
    }
    
 bailout:
 Serial.println("Exiting Homing Procedure"); 
 delay(3*1000);      
}


void controllerStep(stepperMotor *stepper, long steps, float rpm=RPM){
  stepper->setRPM(rpm);
  stepper->move(steps);
  stepper->stepsTaken = stepper->stepsTaken + steps;
}

void controllerRotate(stepperMotor *stepper, float deg, float rpm=RPM){
  long steps = stepper->calcStepsForRotation(deg);
  controllerStep(stepper,steps,rpm);
}

void azimuth(float az){
  long stepNo = stepper1.calcStepsForRotation(az) - (stepper1.stepsTaken)%200;
  controllerStep(&stepper1,stepNo);
}

void elevation(float el){
  long stepNo = stepper2.calcStepsForRotation(el) - (stepper2.stepsTaken)%200;
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
  delay(100);
  while (Serial.available()>0){Serial.read();}
  Serial.println("Unknown Command");
};

void processState(){
  delay(100);
  int axis,stepNo;
  float angle,rpm;
  switch(state){
    case HOME:
      //if ((Serial.read() == '\r')&&(Serial.read() == '\n')) Homing();
      //else error();
      Homing();
      break;
    case STEP:
      axis = Serial.parseInt();
      stepNo = Serial.parseInt();
      rpm = Serial.parseFloat();
      if (rpm==0){rpm=RPM;};

      if (axis == 1){controllerStep(&stepper1,stepNo,rpm);}
      else if (axis == 2){controllerStep(&stepper2,stepNo,rpm);}
      else error();
      
      break;
    case MOVE:
      axis = Serial.parseInt();
      angle = Serial.parseFloat();
      rpm = Serial.parseFloat();
      if (rpm==0){rpm=RPM;};
      
      if (axis == 1){controllerRotate(&stepper1,angle,rpm);}
      else if (axis == 2){controllerRotate(&stepper2,angle,rpm);}
      else error();
      
      break;
    case AZIMUTH: // Remember to ensure that positive convention matches the positive direction for motor rotation
      azimuth(Serial.parseFloat());
      
      break;
    case ELEVATION: // Remember to ensure that positive convention matches the positive direction for motor rotation
      elevation(Serial.parseFloat());
      
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
  delay(100);
  while (Serial.available()>0){Serial.read(); Serial.println("FLUSH");}
};
