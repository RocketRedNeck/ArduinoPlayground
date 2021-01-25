

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GPS.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!
#include <SoftwareSerial.h>

#include <LLAPSerial.h>  // include the library

char deviceId[] = "--";

// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// If using software serial (sketch example default):
//   Connect the GPS TX (transmit) pin to Digital 8
//   Connect the GPS RX (receive) pin to Digital 7
// If using hardware serial:
//   Connect the GPS TX (transmit) pin to Arduino RX1 (Digital 0)
//   Connect the GPS RX (receive) pin to matching TX1 (Digital 1)

// If using software serial, keep these lines enabled
// (you can change the pin numbers to match your wiring):
SoftwareSerial gpsSerial(8, 7);

#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"

// Change GPS baud rate (NOTE: SoftwareSerial maximum is 57600 on Arduino (UNO and Genuino 101)
#define PMTK_SET_NMEA_BAUDRATE_38400 "$PMTK251,38400*27"
#define PMTK_SET_NMEA_BAUDRATE_57600 "$PMTK251,57600*2C"
#define PMTK_SET_NMEA_BAUDRATE_9600  "$PMTK251,0*28"    // Default command is 9600

// turn on only the second sentence (GPRMC)
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
// turn on GPRMC and GGA
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// turn on ALL THE DATA
#define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// turn off output
#define PMTK_SET_NMEA_OUTPUT_OFF "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"

#define PMTK_Q_RELEASE "$PMTK605*31"


// i2c
//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();

// You can also use software SPI
//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(13, 12, 11, 10, 9);
// Or hardware SPI! In this case, only CS pins are passed in
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(10, 9);

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
  Serial.begin(115200); // Hardware serial

  gpsSerial.begin(9600); // Soft serial to gps
  gpsSerial.println(PMTK_SET_NMEA_BAUDRATE_38400);
  gpsSerial.end();
  gpsSerial.begin(38400);

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

bool gpsStarted = false;

void processSchedule(void)
{
  unsigned long aboutNow_ms = millis();
  
  // If cycle mode is enabled then 
  if (cycle)
  {
    if ( ! gpsStarted)
    {     
      // Start the GPS messsages     
      gpsSerial.println(PMTK_Q_RELEASE);
      
      // you can send various commands to get it started
      gpsSerial.println(PMTK_SET_NMEA_OUTPUT_RMCONLY);      // Minimum data only
      //gpsSerial.println(PMTK_SET_NMEA_OUTPUT_RMCGGA);       // Minimum data + GGA
      //gpsSerial.println(PMTK_SET_NMEA_OUTPUT_ALLDATA);
    
      gpsSerial.println(PMTK_SET_NMEA_UPDATE_1HZ);

      gpsStarted = true;
    }
    
    if (aboutNow_ms >= schedTime_ms)
    {
      reply = "T";
      reply += String(float(aboutNow_ms)*1.0e-3, 3);
      LLAP.sendMessage(reply);

      lsm.read();

      // Build a NMEA-like message for this sensor data
      Serial.print("nPPLSM");
      Serial.print(lsm.accelData.x);
      Serial.print(",");
      Serial.print(lsm.accelData.y);
      Serial.print(",");
      Serial.print(lsm.accelData.z);
      Serial.print(",");
      Serial.print(lsm.gyroData.x);
      Serial.print(",");
      Serial.print(lsm.gyroData.y);
      Serial.print(",");
      Serial.print(lsm.gyroData.z);
      Serial.print(",");
      Serial.print(lsm.magData.x);
      Serial.print(",");
      Serial.print(lsm.magData.y);
      Serial.print(",");
      Serial.print(lsm.magData.z);
      Serial.println(",");
    
      // Schedule next
      schedTime_ms += cycleTime_ms;
    }
     
  }
  else if (gpsStarted)
  {
    gpsSerial.println(PMTK_SET_NMEA_OUTPUT_OFF);
    gpsStarted = false;
  }
}

void processGps(void)
{
  // Accumulate message on soft serial while sharing hardware serial line (at the expense of memory)
  // In this case we will wait for a few characters at a time to ensure that
  // we keep up with serial stream, but get out often enough to keep up with other things
  // The soft serial is 9600 baud 8+1+1+N, thus 10-bits transfered per character
  // This is 960 bytes per second, or roughly 1.041 ms per character
  // Grabbing up to 9 characters at a time will keep this branch at roughly 9.375 ms + exit latency
  // This allows other processing to proceed at roughly 100 Hz

  long start = 0;
  bool gpsMessageInProgress = false;
  if (gpsSerial.available())
  {
      char c = gpsSerial.read();
      if (c == '$')
      {
        // embed NMEA string into LLAP stream
        gpsMessageInProgress = true;
        Serial.print('n');
        start = millis();
      } 

      if (gpsMessageInProgress)
      {
        int cnt = 1;
        bool timeout = false;
        while (gpsMessageInProgress && !timeout)
        {
          if (gpsSerial.available())
          {
            c = gpsSerial.read();
          
            if (c == 13)
            {
              // Drop <CR>
            }
            else if (c == 10)
            {
              // <LF> Done!
              gpsMessageInProgress = false;
              Serial.println("");
              //Serial.flush();
            }
            else
            {
               // Forward
               Serial.print(c);
            }
          } // end if serial data available
  
         // Wait no longer than 100 ms, get out and try something else
         timeout = ((millis() - start) > 40);
        }
  
        // Force termination
        if (gpsMessageInProgress)
        {
          gpsMessageInProgress = false;
          Serial.println("");
          //Serial.flush();
        }
      } // end if any message sent
  }
}

void loop() 
{
  processMessage();

  processSchedule();

  processGps();
}




