/*****************************************************************
LSM9DS0_CAL.ino

Hardware setup: This library supports communicating with the
LSM9DS0 over either I2C or SPI. If you're using I2C, these are
the only connections that need to be made:
	LSM9DS0 --------- Arduino
	 SCL ---------- SCL (A5 on older 'Duinos')
	 SDA ---------- SDA (A4 on older 'Duinos')
	 VDD ------------- 3.3V
	 GND ------------- GND
         DRDYG-------------4   (gyro data ready interrupt, can be any digital pin)
         INTX1-------------3   (accelerometer data ready interrupt, can be any digital pin)
         INTX2-------------2   (magnetometer data ready interrupt, can be any digital pin)
(CSG, CSXM, SDOG, and SDOXM should all be pulled high jumpers on 
  the breakout board will do this for you.)
  
 Note: The LSM9DS0 in the I2C mode uses the Arduino Wire library. 
 Because the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
 We have disabled the internal pull-ups used by the Wire library in the Wire.h/twi.c utility file.
 We are also using the 400 kHz fast I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.
  
If you're using SPI, here is an example hardware setup:
	LSM9DS0 --------- Arduino
          CSG -------------- 9
          CSXM ------------- 10
          SDOG ------------- 12
          SDOXM ------------ 12 (tied to SDOG)
          SCL -------------- 13
          SDA -------------- 11
          VDD -------------- 3.3V
          GND -------------- GND
	
The LSM9DS0 has a maximum voltage of 3.6V. Make sure you power it
off the 3.3V rail! And either use level shifters between SCL
and SDA or just use a 3.3V Arduino Pro.	  

In addition, this sketch uses a Nokia 5110 48 x 84 pixel display which requires 
digital pins 5 - 9 described below. If using SPI you might need to press one of the A0 - A3 pins
into service as a digital input instead.

Development environment specifics:
	IDE: Arduino 1.0.5
	Hardware Platform: Arduino Pro 3.3V/8MHz
	LSM9DS0 Breakout Version: 1.0

*****************************************************************/

// The Adafruit LSM9DS0 requires both the SPI and Wire libraries.
// Unfortunately, you'll need to include both in the Arduino
// sketch, before including the SFE_LSM9DS0 library.
#include <SPI.h> // Included for SFE_LSM9DS0 library
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>


// Create LSM9DS0 board instance.
Adafruit_LSM9DS0     lsm(1000);  // Use I2C, ID #1000

void configureLSM9DS0(void)
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

void setup()
{
  Serial.begin(115200); 

  // Initialise the LSM9DS0 board.
  if(!lsm.begin())
  {
    // There was a problem detecting the LSM9DS0 ... check your connections
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }
  
  // Setup the sensor gain and integration time.
  configureLSM9DS0();

}

void loop()
{
  //sensors_event_t accel, mag, gyro, temp;

  lsm.read();
  //lsm.getEvent(&accel, &mag, &gyro, &temp); 
  
  // Print the sensor data
  Serial.print("Raw:");
  Serial.print(lsm.accelData.x,0);
  Serial.print(',');
  Serial.print(lsm.accelData.y,0);
  Serial.print(',');
  Serial.print(lsm.accelData.z,0);
  Serial.print(',');
  Serial.print(lsm.gyroData.x,0);
  Serial.print(',');
  Serial.print(lsm.gyroData.y,0);
  Serial.print(',');
  Serial.print(lsm.gyroData.z,0);
  Serial.print(',');
  Serial.print(lsm.magData.x,0);
  Serial.print(',');
  Serial.print(lsm.magData.y,0);
  Serial.print(',');
  Serial.print(lsm.magData.z,0);
  Serial.println();  
    
  delay(50);
    
}


