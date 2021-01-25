// Shows how to run three Steppers at once with varying speeds
//
// Requires the Adafruit_Motorshield v2 library 
//   https://github.com/adafruit/Adafruit_Motor_Shield_V2_Library
// And AccelStepper with AFMotor support 
//   https://github.com/adafruit/AccelStepper

// This tutorial is for Adafruit Motorshield v2 only!
// Will not work with v1 shields


#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

#include <Servo.h>

#include <AccelStepper.h>

// Actual motor has 32*16.032 = 513.024 (due to gear reduction)
// Only worry about the truncation if we are traveling a long distance
const uint16_t stepsPerRevolution = 32*16.032; // will truncate
const float degreesPerStep = 360.0/(float)stepsPerRevolution;
const float stepsPerDegree = 1.0/degreesPerStep;

const float pi = 3.1415926f;
const float wheelDiameter_mm = 80.0;
const float wheelCircumference_mm = pi * wheelDiameter_mm;
const float mmPerStep = wheelCircumference_mm / (float)stepsPerRevolution;
const float stepsPerMm = 1.0 / mmPerStep;


Adafruit_MotorShield AFMStop(0x60); // Default address, no jumpers
//Adafruit_MotorShield AFMSbot(0x61); // Rightmost jumper closed

// Connect two steppers with correct steps per revolution
// to the top shield
Adafruit_StepperMotor *myStepper1 = AFMStop.getStepper(stepsPerRevolution, 1);
Adafruit_StepperMotor *myStepper2 = AFMStop.getStepper(stepsPerRevolution, 2);

// Connect one stepper with 200 steps per revolution (1.8 degree)
// to the bottom shield
//Adafruit_StepperMotor *myStepper3 = AFMSbot.getStepper(200, 2);

Servo myServo;

// you can change these to DOUBLE or INTERLEAVE or MICROSTEP!
// wrappers for the first motor!
void forwardstep1() {  
  myStepper1->onestep(FORWARD, SINGLE);
}
void backwardstep1() {  
  myStepper1->onestep(BACKWARD, SINGLE);
}
// wrappers for the second motor!
void forwardstep2() {  
  myStepper2->onestep(FORWARD, DOUBLE);
}
void backwardstep2() {  
  myStepper2->onestep(BACKWARD, DOUBLE);
}
// wrappers for the third motor!
void forwardstep3() {  
//  myStepper3->onestep(FORWARD, INTERLEAVE);
  myServo.write(myServo.read() + 1);
}
void backwardstep3() {  
//  myStepper3->onestep(BACKWARD, INTERLEAVE);
  myServo.write(myServo.read() - 1);
}

// Now we'll wrap the 3 steppers in an AccelStepper object
AccelStepper stepper1(forwardstep1, backwardstep1);
AccelStepper stepper2(forwardstep2, backwardstep2);
AccelStepper stepper3(forwardstep3, backwardstep3);

void setup()
{  
   Serial.begin(115200);           // set up Serial library at maximum bps
   Serial.println("Multi-Stepper test!");
 
  //AFMSbot.begin(); // Start the bottom shield
  AFMStop.begin(); // Start the top shield
 
  uint16_t steps = (uint16_t)(45.0 * stepsPerDegree);  // truncate
  
  stepper1.setMaxSpeed(100.0);
  stepper1.setAcceleration(100.0);
  stepper1.moveTo(steps); //was 24);
    
  steps = (uint16_t)(180.0 * stepsPerDegree);    
  stepper2.setMaxSpeed(200.0);
  stepper2.setAcceleration(100.0);
  stepper2.moveTo(steps); // was 50000);

  myServo.attach(10);
  myServo.write(90);
  steps = 45;
  stepper3.setMaxSpeed(10.0);
  stepper3.setAcceleration(10.0);
  stepper3.moveTo(steps);
    
}

void loop()
{
    // Change direction at the limits
    if (stepper1.distanceToGo() == 0)
	stepper1.moveTo(-stepper1.currentPosition());

    if (stepper2.distanceToGo() == 0)
	stepper2.moveTo(-stepper2.currentPosition());

    if (stepper3.distanceToGo() == 0)
	stepper3.moveTo(-stepper3.currentPosition());

    stepper1.run();
    stepper2.run();
    stepper3.run();

}

