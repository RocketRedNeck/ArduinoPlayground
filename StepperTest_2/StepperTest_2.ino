/* 
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2 
---->	http://www.adafruit.com/products/1438
*/


#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Connect a stepper motor with 200 steps per revolution (1.8 degree)
// to motor port #2 (M3 and M4)
//Adafruit_StepperMotor *myMotor = AFMS.getStepper(200, 2);
// mok - actual motor has 32*16.032 = 513.024 (due to gear reduction)
// Only worry about the truncation if we are traveling a long distance
const uint16_t stepsPerRevolution = 32*16.032; // will truncate
const float degreesPerStep = 360.0/(float)stepsPerRevolution;
const float stepsPerDegree = 1.0/degreesPerStep;

Adafruit_StepperMotor *myMotor = AFMS.getStepper(stepsPerRevolution, 2);

const float pi = 3.1415926f;
const float wheelDiameter_mm = 80.0;
const float wheelCircumference_mm = pi * wheelDiameter_mm;
const float mmPerStep = wheelCircumference_mm / (float)stepsPerRevolution;
const float stepsPerMm = 1.0 / mmPerStep;

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Stepper test!");

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  myMotor->setSpeed(25);  // was 10 rpm   
}

void loop()
{
// was
//  Serial.println("Single coil steps");
//  myMotor->step(100, FORWARD, SINGLE); 
//  myMotor->step(100, BACKWARD, SINGLE); 
//
//  Serial.println("Double coil steps");
//  myMotor->step(100, FORWARD, DOUBLE); 
//  myMotor->step(100, BACKWARD, DOUBLE);
//  
//  Serial.println("Interleave coil steps");
//  myMotor->step(100, FORWARD, INTERLEAVE); 
//  myMotor->step(100, BACKWARD, INTERLEAVE); 
//  
//  Serial.println("Microstep steps");
//  myMotor->step(50, FORWARD, MICROSTEP); 
//  myMotor->step(50, BACKWARD, MICROSTEP);

  Serial.println("Moving 90 degrees Forward");
  uint16_t steps = (uint16_t)(90.0 * stepsPerDegree);  // truncate
  myMotor->step(steps, FORWARD, SINGLE);
  
  delay(2000);
  
  Serial.println("Moving 90 degrees Backward");
  myMotor->step(steps, BACKWARD, SINGLE);

  delay(2000);
  
  Serial.println("Moving 25.4 mm Forward");
  steps = (uint16_t)(25.4 * stepsPerMm);
  myMotor->step(steps, FORWARD, SINGLE);

  delay(2000);
  
  Serial.println("Moving 25.4 mm Backward");
  myMotor->step(steps, BACKWARD, SINGLE);

  delay(2000);

  Serial.println("Moving 1 mm Forward");
  steps = (uint16_t)(1.0 * stepsPerMm);
  myMotor->step(steps, FORWARD, SINGLE);

  delay(2000);
  
  Serial.println("Moving 1 mm Backward");
  myMotor->step(steps, BACKWARD, SINGLE);

  delay(2000);

}
