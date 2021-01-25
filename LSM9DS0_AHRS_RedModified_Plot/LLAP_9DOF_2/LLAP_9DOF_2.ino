

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!
#include <LLAPSerial.h>  // include the library

char deviceId[] = "--";


// i2c
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();

// You can also use software SPI
//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(13, 12, 11, 10, 9);
// Or hardware SPI! In this case, only CS pins are passed in
//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(10, 9);

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


void setup() 
{
  // initialise serial:
  Serial.begin(115200);
  
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

bool cycle = false;
unsigned long count = 0;
unsigned long cycleTime_ms = 1000;
unsigned long schedTime_ms = cycleTime_ms;

void processMessage(void)
{
  // print the string when a newline arrives:
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
  }   // end if message received
} // end processMessage

void processSchedule(void)
{
  unsigned long aboutNow_ms = millis();
  
  // If cycle mode is enabled then 
  if (cycle)
  {
    if (aboutNow_ms >= schedTime_ms)
    {
      reply = "TICK";
      reply += count++;
      LLAP.sendMessage(reply);

      lsm.read();

      // Build a NMEA-like message for this sensor data
      reply = "PILSM" + 
               String(lsm.accelData.x * aLsb, 5) + ',' +
               String(lsm.accelData.y * aLsb, 5) + ',' +
               String(lsm.accelData.z * aLsb, 5) + ',' +
               String(lsm.gyroData.x * gLsb, 5) + ',' +
               String(lsm.gyroData.y * gLsb, 5) + ',' +
               String(lsm.gyroData.z * gLsb, 5) + ',' +
               String(lsm.magData.x * mLsb, 5) + ',' +
               String(lsm.magData.y * mLsb, 5) + ',' +
               String(lsm.magData.z * mLsb, 5) + ',';
               
      LLAP.sendExtendedMessage(reply);
    
      // Schedule next
      schedTime_ms += cycleTime_ms;
    }
     
  }  
}

void loop() 
{
  processMessage();

  processSchedule();
}




