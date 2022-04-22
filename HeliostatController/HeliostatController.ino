/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                                                                                     *
 *  Code name:             HeliostatController.ino                                     *
 *  Authors:               Helio-Tide                                                  *
 *  University:            University of Florida                                       *
 *  Group Number:          448L                                                        *
 *  Academic year:         2021/2022                                                   *
 *  Board:                 SparkFun ESP32 Thing                                        *
 *                                                                                     *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/*
HelioTide Spring 2022
Notes:
  Motor Info
    Coils for Motor
    A+ Black
    A- Green
    B+ Red
    B- Blue
    Motor Rotation Direction -  With mounting face towards observer
    + = CCW
    - = CW


Future Suggestions:
  Input validation
  Solving equations within the code
*/

#include <HardwareSerial.h>
#include <Arduino.h>
#include "A4988.h"
#include "BasicStepperDriver.h"
#include "Adafruit_SHTC3.h"

//Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step---With microstepping, it now takes 16000 steps to rotate 360 degrees
#define MOTOR_STEPS 200
#define RPM 5

// Since microstepping is set externally, make sure this matches the selected mode
// If it doesn't, the motor will move at a different RPM than chosen
// 1=full step, 2=half step etc.
#define MICROSTEPS 16

// All the wires and Pin numbers on microcontroller needed for full functionality
#define STEP1 14
#define DIR1 32

#define STEP2 15
#define DIR2 33

#define SDAPIN 23//For I2C Comms
#define SCLPIN 22

#define LIMITSWITCH1 13 //Azimuth
#define LIMITSWITCH2 12 //Elevation 1
#define LIMITSWITCH3 27 //Elevation 2


//2-wire basic config, microstepping is hardwired on the driver. Creating Stepper object
class stepperMotor : public BasicStepperDriver {
public:  
  long stepsTaken=0; //Keeps track of steps taken from home position on Motor
  byte dirPin;
  byte stepPin;
  byte limitSwitchPin1;byte limitSwitchPin2;

  stepperMotor(byte dirPin, byte stepPin, byte limitPin1, byte limitPin2) : BasicStepperDriver(MOTOR_STEPS, dirPin, stepPin){
    this->limitSwitchPin1 = limitPin1;
    this->limitSwitchPin2 = limitPin2;
    pinMode(limitSwitchPin1, INPUT_PULLUP); //initialize limit switch pin
    pinMode(limitSwitchPin2, INPUT_PULLUP);
  }
};

stepperMotor stepper1(DIR1, STEP1, LIMITSWITCH1,999); //Azimuth Motor
stepperMotor stepper2(DIR2, STEP2, LIMITSWITCH2, LIMITSWITCH3); //Elevation Motor

Adafruit_SHTC3 shtc3 = Adafruit_SHTC3();//Temperature & Humidity Sensor

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
void controllerStep(stepperMotor *stepper, long steps, float rpm=RPM);
void controllerRotate(stepperMotor *stepper, float deg, float rpm=RPM);
 
// the setup function runs once when you press reset or power the board
void setup() {
  // initialize serial communication:
  Serial.begin(115200);
  state = NONE;

  //initialize stepper motor
  stepper1.begin(RPM, MICROSTEPS);  
  stepper2.begin(RPM, MICROSTEPS); 

  Wire.begin(SDAPIN,SCLPIN);
  if (! shtc3.begin()) {
    Serial.println("Couldn't find SHTC3");
    while (1) delay(1);
  }
  Serial.println("Found SHTC3 sensor");
  
  Serial.println("Controller is Ready"); 
}




// the loop function runs over and over again forever
void loop(){
  while (Serial.available()>0){
      processData(Serial.read());
  };
  
  Serial.print((String)"Stepper_1 Steps Taken: " + stepper1.stepsTaken + "\tAzimuth: ");
  Serial.println(stepper1.stepsTaken/16000.*360 - 0.2,5);
  Serial.print((String)"Stepper_2 Steps Taken: " + stepper2.stepsTaken + "\tElevation: ");
  Serial.println((180-(stepper2.stepsTaken/16000.*360)) + 0.2,5);
  delay(0.1*1000);    
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
        controllerStep(&stepper1,2,30);delay(2*300/MICROSTEPS/30);//Rapid Movement 
        if (digitalRead(stepper1.limitSwitchPin1) == LOW){ //Assuming Normally Open limit switch, when the pin is low the switch is closed and the pin will read low. When it reads low, the homing position is achieved
          Home1_init = true;
          Serial.println("Stepper 1 Rapid Homing Complete.");  
        }
  
      }//End of rapid homing
      Serial.println("Initializing Stepper 1 Precision Homing");
      delay(1*1000);
      controllerStep(&stepper1,-20*MICROSTEPS,5);delay(500);//Backing up for Precision Movement
      while(digitalRead(stepper1.limitSwitchPin1) == HIGH){//Maybe switch bounce or floating voltage is an issue
        controllerStep(&stepper1,2);delay(2*300/MICROSTEPS/5);//Precision Movement
      }
      delay(50);//Button Debounce
      Serial.println("Home of stepper 1 Found");
      Home1_final=true;
      //For future changes. Adjust homing such that at 0 zero steps taken for each motor represents 0 elevations and 0 azmituth (to represtent by marker on heliostat)

      //Uncomment and adjust x to give a new 0 position
      //controllerStep(&stepper1,x,120); 
      stepper1.stepsTaken = 0;
    }

   
    Time0 = millis();
    while (!Home2_final){
      while(!Home2_init){
        if (millis()-Time0 >= TimeErr){
          Serial.println("Error 404: Home Not Found...I'm Lost, Help");
          goto bailout;// Skips to end of homing procedure to end it.
        }
        controllerStep(&stepper2,-2,30);delay(2*300/MICROSTEPS/30);//Rapid Movement
        if (digitalRead(stepper2.limitSwitchPin1) == LOW){ //Assuming Normally Open limit switch, when the pin is low the switch is closed and the pin will read low. When it reads low, the homing position is achieved
          Home2_init = true;
          Serial.println("Stepper 2 Rapid Homing Complete.");  
        }
  
      }//End of rapid homing
      Serial.println("Initializing Stepper 2 Precision Homing");
      delay(1*1000);
      controllerStep(&stepper2,20*MICROSTEPS);delay(500);//Backing up for Precision Movement
      while(digitalRead(stepper2.limitSwitchPin1) == HIGH){//Maybe switch bounce or floating voltage is an issue
        controllerStep(&stepper2,-2);delay(2*300/MICROSTEPS/5);//Precision Movement
      }
      delay(50);//Button Debounce
      Serial.println("Home of stepper 2 Found");
      Home2_final=true;
      //For future changes. Adjust homing such that at 0 zero steps taken for each motor represents 0 elevations and 0 azmituth (to represtent by marker on heliostat)

      //Uncomment and adjust x and elevationOffset to give a new 0 position
      //float elevationOffset = 78.9;//Angle of mirror with respect to the mounting surface when the elevation motor has activated its limit switch. Adjust as needed.
      //stepper2.stepsTaken = stepper2.calcStepsForRotation((90-elevationOffset)*5)-x;
      //controllerStep(&stepper2,stepper2.calcStepsForRotation(90.*5) - (stepper2.stepsTaken)%(200*MICROSTEPS*5),120);
      
  
    }
    
 bailout: 
 Serial.println("Exiting Homing Procedure"); 
 delay(3*1000);      
}


void controllerStep(stepperMotor *stepper, long steps, float rpm){
  stepper->setRPM(rpm);
  stepper->move(steps);
  stepper->stepsTaken = stepper->stepsTaken + steps;
}

void controllerRotate(stepperMotor *stepper, float deg, float rpm){
  long steps = stepper->calcStepsForRotation(deg*5);
  controllerStep(stepper,steps,rpm);
}


void azimuth(float az){
  long stepNo = stepper1.calcStepsForRotation(az*5) - (stepper1.stepsTaken)%(200*MICROSTEPS*5);
  controllerStep(&stepper1,stepNo);
  long corraz = -0.51582*az+23.67173;
  delay(500);
  controllerStep(&stepper1,corraz);
}


void elevation(float el){
  el = 180-el;
  long stepNo = stepper2.calcStepsForRotation(el*5) - (stepper2.stepsTaken)%(200*MICROSTEPS*5);
  controllerStep(&stepper2,stepNo);
  long correl = 2.435856*(el)-303.57;
  delay(500);
  controllerStep(&stepper2,correl);
}

void danger(){
Serial.println("Danger: Moving to safe position"); 
elevation(90);
azimuth(90);//Honestly probably not necessary since the mirror is flat....but it's here if needed.
delay(5*1000);
};

void temperature(){
  sensors_event_t humidity, temp;
  shtc3.getEvent(&humidity, &temp);
  
  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" C");

  Serial.print("Humidity: "); Serial.print(humidity.relative_humidity); Serial.println("% rH");
  delay(5*1000);
  
};

void powerDown(){
  Serial.println("This is the Power Down Stub");  
  delay(5*1000); 
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
    case TEMP:
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

//ZG signing off
