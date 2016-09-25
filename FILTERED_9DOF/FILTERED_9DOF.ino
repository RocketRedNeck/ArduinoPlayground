// ****************************************************************************************************
//
// FILTERED_9DOF
//
// Example for using a 9 degrees-of-freedom sensor with a filtered estimate of angle
// from the gyro rates. 
//
// NOTE: If you need more flash memory for code, eliminate the LLAP interface
// and just set up a set of constants to drive the loop as desired (default).
//
// This variant uses the MicroTimer interface (micros()) to keep track of time
// to microsecond precision (4 microsecond resolution on 16 MHz UNO).
//
// This demonstrates that the I2C read of the sensor takes just over 4 milliseconds
// and the serial output (ASCII) takes just over 3.7 milliseconds.
//
// The 100 Hz loop is stable to roughly 30 microseconds or better indicating that
// any remaining calculations within this context cannot exceed 2.3 milliseconds
// or the period would be disrupted.
//
// This indicates that if more time is needed for computation then there are some
// sources to attack:
//    1) reduce the serial output by performing less formatting
//    2) Switch to hardware SPI interface
//
// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------
// Copyright (c) 2016 - RocketRedNeck.com RocketRedNeck.net
//
// RocketRedNeck and MIT Licenses
//
// RocketRedNeck hereby grants license for others to copy and modify this source code for
// whatever purpose other's deem worthy as long as RocketRedNeck is given credit where
// where credit is due and you leave RocketRedNeck out of it for all other nefarious purposes.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// ****************************************************************************************************
 
// ----------------------------------------------------------------------------------------------------
// Some constants are herein #defined or enumerated rather than instantiated as const to avoid using 
// memory unless the values are actually used in the code below. Mileage depends on quality of linker
// and rather than tempt fate we just revert to old habits.
// ----------------------------------------------------------------------------------------------------



#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!

#include <LLAPSerial.h>       // include the library from https://github.com/CisecoPlc/LLAPSerial 
                              // place in Arduino\Library path
                              // Useful to build up a device control/status pattern 

#include "MicroTimer.h"       // Keep track of one or more markable times 

char deviceId[] = "--";

// i2c
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();

// You can also use software SPI
//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(13, 12, 11, 10, 9);
// Or hardware SPI! In this case, only CS pins are passed in
//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(10, 9);

// SPI NOTES from Sparkfun
// Connect CSG and CSXM to two individually controllable pins on your microcontroller. 
// These chip-selects are active-low – when the pin goes LOW, SPI communication with either the gyro (CSG) or accel/mag (CSXM) is enabled.
//
// SDOG and SDOXM are the serial data out pins. 
// In many cases you’ll want to connect them together, and wire them to your microcontroller’s MISO (master-in, slave-out) pin.
//
// Connect SCL to your microcontroller’s SCLK (serial clock) pin.
// Connect SDA to your microcontroller’s MOSI (master-out, slave-in) pin.


float aLsb = 2.0 / 32768.0;
float gLsb = 245.0 / 32768.0;
float mLsb = 2.0 / 32768.0;

void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}

// Default this application to start transmitting at 100 Hz as soon as it
// starts
bool cycle = false;
unsigned long count = 0;
unsigned long cycleTime_ms = 10;
unsigned long long cycleTime_us = cycleTime_ms * 1000;
long long schedTime_us = cycleTime_us;

// Create a single global timer to keep track of time
// since power up
MicroTimer timer;

void setup() 
{
  // initialise serial:
  Serial.begin(115200); // Hardware serial

  // Initialise the LLAPSerial library and set our default ID to -- (unknown)
  LLAP.init(deviceId);

  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    // Error #0001 is our first error; it indicates that the LSM sensor is not wired correctly
    LLAP.sendMessage(String("ERROR0001"));
    while (1);
  }
  else
  {
     LLAP.sendMessage(String("STARTED"));
  }
}

String msg;
String reply;

void processMessage(void)
{
  // process the LLAP command when a newline arrives:
  if (LLAP.bMsgReceived) 
  {
    msg = LLAP.sMessage;
    reply = msg;
    LLAP.bMsgReceived = false;  // if we do not clear the message flag then message processing will be blocked

    // HELLO and CHDEVID are handled by the LLAP message processing automatically
    // and do NOT respond with the device ID
    if (msg.compareTo("ACK------") == 0)
    {
      LLAP.sendMessage(reply);
    }
    else if (msg.compareTo("DEVNAME--") == 0)
    {
      reply = "ARDUINO--";
      LLAP.sendMessage(reply);
    }
    else if (msg.compareTo("CYCLE----") == 0)
    {
      cycle = true;
    }
    else if (msg.compareTo("STOP-----") == 0)
    {
      cycle = false;
    }
    else if (msg.startsWith("INTVL"))
    {
      long intvl = msg.substring(5).toInt();

      if (intvl > 0)
      {
        count = 0;
        switch (msg.substring(8).c_str()[0])
        {
          case 'T':
            cycleTime_ms = intvl;
            break;
          case 'S':
            cycleTime_ms = (intvl * 1000);
            break;
          case 'M':
            cycleTime_ms = (intvl * 60000);
            break;
          case 'H':
            cycleTime_ms = (intvl * 36000000);
          case 'D':
            cycleTime_ms = (intvl * 86400000);
            break;
          default:
            break;
        } // end switch on interval scale
        cycleTime_us = cycleTime_ms * 1000;

        // Align the scheduled time to the interval boundary
        // There is no compelling reason other than to make the
        // display increment with as little remainder as possible
        // This just makes it easier to see the chosen rate by
        // seeing which digit is changing. A little extra time
        // to compute for a little easier debug information.
        schedTime_us = schedTime_us - (schedTime_us % cycleTime_us) + cycleTime_us;
      } // end if interval is positive
    } // end if recognized command
    else
    {
      // Define a generic error message for unknown
      // commands; the offending message is sent back
      reply = "ERROR----";
      LLAP.sendMessage(reply);
      LLAP.sendMessage(msg);
    }
  }   // end if message received by LLAP (hardware serial)
} // end processMessage

void processSchedule(void)
{
  // NOTE: Using millis() in this scheduler will jitter
  // about 1 ms depending upon the number of various
  // "tasks" to be performed. Schedule misses will occur if
  // the "tasks" exceed the scheduled period.
  //
  // NOTE: Resolution of the schedule can be improved with
  // micros() but will require an extending the result to
  // keep a sufficiently large value of accumulated time,
  // if a concept of "absolute time" is needed, which is
  // useful if something must be scheduled to occur at
  // some point in the future (like a one-shot event).
  long long aboutNow_us = timer.read();
  
  // If cycle mode is enabled then 
  if (cycle)
  {    
    if (aboutNow_us >= schedTime_us)
    {
      long long beforeRead_us = timer.read();
      
      lsm.read();

      long long afterRead_us = timer.read();

      // Build a NMEA-like message for this sensor data
      // We prefix the message with 'n' to distinquish
      // it from the LLAP messages (which are too short
      // to create an efficient stream); the truth is
      // that even an NMEA steam is heavy on the bus
      // requiring transmission of up to 7 bytes for
      // values that only require 2 bytes. Further, the
      // Serial.print(...) functions, while convenient
      // put a large burden on the available processing
      // capacity when formatting is involved. Future
      // designs should consider a more direct transfer
      // of bytes with formatting performed by processors
      // with greater capacity (this will leave more
      // of the arduino environment for filters that
      // depend on latency knowledge to produce accurate
      // answers).
      //
      // For now we send the data formatted to the serial
      // stream to make it somewhat human readable.
      //
      // In our design we will include measures of time
      // to estimate latency of read and print operations
      //
      Serial.print("nPPLSM,");
      timer.print(aboutNow_us);
      Serial.print(",");
      timer.print(beforeRead_us);
      Serial.print(",");
      timer.print(afterRead_us);
      Serial.print(",");
      Serial.print(lsm.accelData.x,0);
      Serial.print(",");
      Serial.print(lsm.accelData.y,0);
      Serial.print(",");
      Serial.print(lsm.accelData.z,0);
      Serial.print(",");
      Serial.print(lsm.gyroData.x,0);
      Serial.print(",");
      Serial.print(lsm.gyroData.y,0);
      Serial.print(",");
      Serial.print(lsm.gyroData.z,0);
      Serial.print(",");
      Serial.print(lsm.magData.x,0);
      Serial.print(",");
      Serial.print(lsm.magData.y,0);
      Serial.print(",");
      Serial.print(lsm.magData.z,0);
      Serial.print(",");
      timer.print();
      Serial.println(",");
    
      // Schedule next
      // Align the scheduled time to the interval boundary
      // There is no compelling reason other than to make the
      // display increment with as little remainder as possible
      // This just makes it easier to see the chosen rate by
      // seeing which digit is changing. A little extra time
      // to compute for a little easier debug information.
      schedTime_us = schedTime_us - (schedTime_us % cycleTime_us) + cycleTime_us;
    }
  }
}

void loop() 
{
  processMessage();

  processSchedule();

}





