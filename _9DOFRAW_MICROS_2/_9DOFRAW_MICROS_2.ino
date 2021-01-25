// ****************************************************************************************************
//
// _9DOFRAW_MICROS
//
// Example for using a 9 degrees-of-freedom sensor for gyro data
//
// This variant uses the MicroTimer interface (micros()) to keep track of time
// to microsecond precision (4 microsecond resolution on 16 MHz UNO).
//
// This demonstrates that the I2C read of the sensor takes just over 1.1 milliseconds
// to read just the gyro data; output is in ASCII to serial port at 250000 baud and
// consumes and additional 2.2 ms
//
// The 100 Hz loop is stable to roughly 24 microseconds or better indicating that
// any remaining calculations within this context cannot exceed 6.7 milliseconds
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

#include "MicroTimer.h"       // Keep track of one or more markable times 

// i2c
Adafruit_LSM9DS0 lsm;//= Adafruit_LSM9DS0();

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
float gLsb = 2000.0 / 32768.0;
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
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}

// Default this application to start transmitting at 100 Hz as soon as it
// starts
bool cycle = true;
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
  Serial.begin(250000); // Hardware serial

  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    while (1);
  }

  setupSensor();
}

sensors_event_t accel, mag, gyro, temp;

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
      
      //lsm.readGyro();
      lsm.getEvent(&accel, &mag, &gyro, &temp);

      long long afterRead_us = timer.read();

      // Build a NMEA-like message for this sensor data
      // For now we send the data formatted to the serial
      // stream to make it somewhat human readable.
      //
      // In our design we will include measures of time
      // to estimate latency of read and print operations
      //
//      Serial.print("$PPLSM,");
//      timer.print(aboutNow_us);
//      Serial.print(",");
//      timer.print(beforeRead_us);
//      Serial.print(",");
//      timer.print(afterRead_us);
//      Serial.print(",");
      
      Serial.print(gyro.gyro.x,0);
      Serial.print(",");
      Serial.print(gyro.gyro.y,0);
      Serial.print(",");
      Serial.println(gyro.gyro.z,0);
//      Serial.print(",");
//      timer.println();
    
      // Schedule next
      schedTime_us += cycleTime_us;
    }
  }
}

void loop() 
{
  processSchedule();

}




