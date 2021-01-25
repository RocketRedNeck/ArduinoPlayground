/********************************************************
Flush

OBJECTIVE:to flush the toilet automaticly or whenever>+> 
         <+<the button is pressed:

by Primo

PROTOTYPE/BREADBOAD VERSION

current circut;
  PING))) sensor SIG pin to D11 (& +5v,gnd)
  1 pushbutton conected to D10 (& pull-up 10K resistor,
  +5v,gnd(conector must be on resistor side...))
  1 led conected to D12 (& 330 resistor,gnd)
  1 servo conected to D9 (& +5v,gnd){power requirements 
  will vary toilet to toilet}
  
// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------
// Copyright (c) 2013 - RocketRedNeck.net
// RocketRedNeck hereby grants license for others to copy and modify this source code for
// whatever purpose other's deem worthy as long as RocketRedNeck is given credit where
// where credit is due and you leave RocketRedNeck out of it for all other nefarious purposes.

********************************************************/
//global variables.......................................
const int pingPin = 11;  //PING)))
const int ledPin = 12;  //led
const int buttPin = 10;  //button
unsigned int duration, inches, button ;  //nonvalued 
                                         //variables, 
                                         //space for 
                                         //numeric values

#include <Servo.h>  //the servo files
Servo handle;  // give a name to the servo,handle
int pos =5;  //tell Arduino what angle to start at 

void setup()    // the first thing done once, for comands
                // to setup some states for void loop
{
  Serial.begin(9600);  //establish Serial conection at
                       //9600 Baud rate
  pinMode (buttPin, INPUT);  //set button to input
  pinMode (ledPin, OUTPUT);  //set led to output 
  handle.attach(9);  //"attach" the handle,er,servo to pin
                     //9
                     
  handle.write(pos);
}

int averageDistance_inches(int numSamples)  //make a function
{
  pinMode(pingPin, OUTPUT);          // Set pin to OUTPUT
  digitalWrite(pingPin, LOW);        // Ensure pin is low
  delayMicroseconds(2);

  unsigned int duration = 0;
  unsigned int inches = 0;  
  for (int i = 0; i < numSamples; ++i) // loop the sample count to 
                                       // average
  {
    pinMode(pingPin, OUTPUT);          // Set pin to OUTPUT    
    digitalWrite(pingPin, HIGH);       // Start ranging
    delayMicroseconds(10);             //   with 10 microsecond burst
    digitalWrite(pingPin, LOW);        // End ranging
    
    pinMode(pingPin, INPUT);           // Set pin to INPUT
    duration = pulseIn(pingPin, HIGH); // Read echo pulse
    inches += duration / 74 / 2;        // Convert to inches
    
    // Set this loop up to sample as quickly as possible
    // Speed of sound is slightly under 1000 ft/s
    // Sensor range is between 0 and 10 feet which means
    // that we need to sample no faster than every 10 ms
    // to avoid overlapping pulses. We will sample every
    // 20 ms to be certain that we have waited long enough
    delay(20);		               // Short delay
  }
  
  return inches/numSamples;  // return the distance to the loop when
                              // the function is called
}

enum States 
{
  WAITING_FOR_CUSTOMER,
  CUSTOMER_DETECTED,
  WAITING_FOR_CUSTOMER_TO_LEAVE,
  EMPTY_STALL_DETECTED,
  FLUSHING,
  DEBOUNCE
};
//states for the switch_case 

States currentState = WAITING_FOR_CUSTOMER; //because we are
States prevState = currentState;    //used if button is pressed
long stateTime_ms = 0;
long buttTime_ms = 0;
const long MIN_PRESENT_TIME_MS   = 3000;  //how long customer must be 
                                          //present to know they're
                                          //actualy here..............                                    
const long MIN_DEPARTURE_TIME_MS = 5000;  //how long customer must be
                                          //gone to know they're
                                          //actualy gone..............
const long MIN_FLUSH_TIME_MS     = 5000;  //how long it takes to flush
const int MAX_PRESENT_DISTANCE_INCHES = 24; //have to be this close to
                                            //be here
const int MIN_DEPARTURE_DISTANCE_INCHES = 60; //have to be this far to
                                              //be gone
const long MIN_BUTT_TIME = 100;           //how long button must be 
                                          //pressed 

void loop()     //  loop will be repeated FOR-E-VER.
{
  int currentDistance_inches = averageDistance_inches(10);  //call that
                                                            //command
                                                            //again
  button = digitalRead(buttPin);    //read the button
  button =! button;                 //reverse because it's pull-up
  switch (currentState)    //switch :case,case,case.
  {
    case WAITING_FOR_CUSTOMER:
    {
      digitalWrite(ledPin,LOW);  //off
      if (currentDistance_inches < MAX_PRESENT_DISTANCE_INCHES)
      {
         stateTime_ms = millis();  //record time
         currentState = CUSTOMER_DETECTED;  //new state
         Serial.print("Customer Detected...\n");  //print
      }  //i see customer
      
      if (button == HIGH)
      {
        buttTime_ms = millis();
        prevState = currentState;  //in case of faulty butt jiggles
        currentState = DEBOUNCE;
      }
      break;  //end of case 1
    }
    case CUSTOMER_DETECTED:
    {
      if (currentDistance_inches < MAX_PRESENT_DISTANCE_INCHES)
      {
         if (millis() - stateTime_ms > MIN_PRESENT_TIME_MS)
         {
           // Customer has been present for minimum required time
           stateTime_ms = millis();
           currentState = WAITING_FOR_CUSTOMER_TO_LEAVE;
           Serial.print("Waiting for Customer to Leave...\n");
           digitalWrite(ledPin,HIGH);  //on
         }
      }
      else  //if not as above,do below
      {
         stateTime_ms = millis();
         currentState = WAITING_FOR_CUSTOMER;
         Serial.print("False Alarm... No Customer\n");
         digitalWrite(ledPin,LOW);
       }
      
      if (button == HIGH)
      {
        buttTime_ms = millis();
        prevState = currentState;
        currentState = DEBOUNCE;
      }
      break;  //end case 2
    }
    case WAITING_FOR_CUSTOMER_TO_LEAVE:
    {
      if (currentDistance_inches > MIN_DEPARTURE_DISTANCE_INCHES)
      {
        stateTime_ms = millis();
        currentState = EMPTY_STALL_DETECTED;
        Serial.print("Empty Stall...\n");
      }
      
      if (button == HIGH)
      {
        buttTime_ms = millis();
        prevState = currentState;
        currentState = DEBOUNCE;
      }
      break;  //end case 3
    }
    case EMPTY_STALL_DETECTED:
    {
      if (currentDistance_inches > MIN_DEPARTURE_DISTANCE_INCHES)
      {
         if (millis() - stateTime_ms > MIN_DEPARTURE_TIME_MS)
         {
           // Stall has been empty for required time
           stateTime_ms = millis();
           currentState = FLUSHING;
           Serial.print("Flushing...\n");
           digitalWrite(ledPin,LOW);
         }
      }
      else
      {
         stateTime_ms = millis();
         currentState = WAITING_FOR_CUSTOMER_TO_LEAVE;
         Serial.print("False Alarm...Customer Still Here\n");
         digitalWrite(ledPin,HIGH);
         
      } 
      
      if (button == HIGH)
      {
        buttTime_ms = millis();
        prevState = currentState;
        currentState = DEBOUNCE;
      }   
      break;  //end case 4
    }
    case FLUSHING:
    {
       if (millis() - stateTime_ms > MIN_FLUSH_TIME_MS)
       {
         // Flush has completed
         pos=5;  //set servo angle to 5 degres
         handle.write(pos);  //write it to that
         stateTime_ms = millis();
         currentState = WAITING_FOR_CUSTOMER;
         Serial.print("Flushing Complete\n\n");
         //Serial.println(button);
       }
       else
       {
         pos=90;
         handle.write(pos);
       }
       //Serial.println(pos);
       break;  //end case 5
    }
    case DEBOUNCE:
    {
     Serial.print("DEBOUNCE...\n\n");
     if (((millis() - buttTime_ms) >= MIN_BUTT_TIME) && 
         (button == HIGH))
     {
      currentState = FLUSHING;
      Serial.print("Flushing...\n");
     }
      else if (((millis() - buttTime_ms) < MIN_BUTT_TIME) &&
         ( button == HIGH))  //if not as above, but like this(), do below
     {
     }  
      else
     {
      currentState = prevState;
      digitalWrite(ledPin,LOW);
     } 
     stateTime_ms=millis();     
 
  } // end switch on current state
}
}
