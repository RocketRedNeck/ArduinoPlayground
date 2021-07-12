#include <Arduino.h>

// See this for debug notes: https://docs.platformio.org/en/latest/plus/debug-tools/avr-stub.html
// Summary:
//      Add avr-debugger library to the project
//      Add the following to the plarform.ini
//          debug_port = SERIAL_PORT (where SERIAL_PORT is something like COM3, where arduino appears)
//          debug_tool = avr-stub
//      Add the GDB debug init to the setup() function
//          debug_init();
//      Include the following file
#include "avr8-stub.h"

// ****************************************************************************************************
//
// RadioTime
//
// Processes the Atomic Clock radio signal (WWVB) and displays time on Nixie via shift register and
// bcd/dec decoder
// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------
// Copyright (c) 2021 - Michael Kessel, Rocket RedNeck
// RocketRedNeck hereby grants license for others to copy and modify this source code for
// whatever purpose other's deem worthy as long as RocketRedNeck is given credit where
// where credit is due and you leave RocketRedNeck out of it for all other nefarious purposes.
// ****************************************************************************************************

#include "MicroTimer.h"
#include <RTClib.h>
#include <time.h>

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// Connect the GPS TX (transmit) pin to Digital 8
// Connect the GPS RX (receive) pin to Digital 9

// ----------------------------------------------------------------------------------------------------
// Some constants are herein #defined or enumerated rather than instantiated as const to avoid using
// memory unless the values are actually used in the code below. Mileage depends on quality of linker
// and rather than tempt fate we just revert to old habits.
// ----------------------------------------------------------------------------------------------------

// ----------------------------------------------------------------------------------------------------
// Pin assignments - enumeration simply to group them nicely
// NOTE: built-in and PWMs can be allocated for any purpose, and as such the comments simply keep
// thing visible to readers so we understand any potential conflicts. Read the specs to see what
// other functions overlap the
// ----------------------------------------------------------------------------------------------------
enum DigitalPins
{
    DP_RX = 0, // Arduino built in function
    DP_TX = 1, // Arduino built in function

    DP_SR_OE_LOW        = 2,    // ~OE,    Pin 13 on 74HC595
    DP_SR_RCLK          = 3,    // RCLK,   Pin 12 on 74HC595, PWM
    DP_SR_SRCLR_LOW     = 4,    // ~SRCLR, Pin 10 on 74HC595
    DP_SR_SRCLK         = 5,    // SRCLK,  Pin 11 on 74HC595, PWM
    DP_SR_DATA          = 6,    // SER,    Pin 14 on 74HC595, PWM

    DP_WWVB_INVERTED    = 7,

    DP_GPS_RX           = 8,    // GPS RX is our TX
    UNUSED9             = 9,    // PWM
    DP_UNUSED10         = 10,   // PWM
    DP_UNUSED11         = 11,   // PWM
    DP_GPS_TX           = 12,   // GPS TX is our RX

    DP_BUILT_IN_LED     = 13, // Arduino built in function

    DP_MAX_NUMBER_OF_PINS,
    DP_BOGUS // A value to recognize when none of the above is indicated.
};

// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------
enum AnalogPins
{
    AP_RANDOM_SEED = 0, // See notations where this is used, below
    AP_UNUSED1 = 1,
    AP_UNUSED2 = 2,
    AP_UNUSED3 = 3,
    AP_UNUSED4 = 4,
    AP_UNUSED5 = 5,

    AP_MAX_NUMBER_OF_PINS,
    AP_BUGUS

};

#define EVER (;;)
#define DIM(x) (sizeof(x) / sizeof(x[0]))
#define LAST(x) (DIM(x) - 1)

// ----------------------------------------------------------------------------------------------------
// Finally! The setup routine runs once at power-on/reset:
// ----------------------------------------------------------------------------------------------------

// you can change the pin numbers to match your wiring:
// Args: GPS TX-->Arduino RX, GPS RX --> Arduino TX
SoftwareSerial mySerial(DP_GPS_TX, DP_GPS_RX);
Adafruit_GPS GPS(&mySerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  false

void updateShiftRegister(byte msw, byte mid, byte lsw)
{
    digitalWrite(DP_SR_RCLK, LOW);
    shiftOut(DP_SR_DATA, DP_SR_SRCLK, MSBFIRST, msw);
    shiftOut(DP_SR_DATA, DP_SR_SRCLK, MSBFIRST, mid);
    shiftOut(DP_SR_DATA, DP_SR_SRCLK, MSBFIRST, lsw);
    digitalWrite(DP_SR_RCLK, HIGH);
}

void setup()
{
    // initialize GDB stub
    //debug_init();

    Serial.begin(115200);

    // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
    GPS.begin(9600);

    pinMode(DP_BUILT_IN_LED,    OUTPUT);

    pinMode(DP_SR_OE_LOW,       OUTPUT);
    pinMode(DP_SR_RCLK,         OUTPUT);
    pinMode(DP_SR_SRCLR_LOW,    OUTPUT);
    pinMode(DP_SR_SRCLK,        OUTPUT);
    pinMode(DP_SR_DATA,         OUTPUT);

    digitalWrite(DP_SR_OE_LOW,      0);
    digitalWrite(DP_SR_SRCLR_LOW,   1);

    // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

    // uncomment this line to turn on only the "minimum recommended" data
    //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
    // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
    // the parser doesn't care about other sentences at this time

    // Set the update rate
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
    // For the parsing code to work nicely and have time to sort thru the data, and
    // print it out we don't suggest using anything higher than 1 Hz

    // Request updates on antenna status, comment out to keep quiet
    //GPS.sendCommand(PGCMD_ANTENNA);

    delay(1000);

    // Ask for firmware version
    mySerial.println(PMTK_Q_RELEASE);

    for (int i = 9; i >= 0; --i)
    {
        updateShiftRegister((i << 4) | i, (i << 4) | i, (i << 4) | i);
        delay(500);
    }
}

// ----------------------------------------------------------------------------------------------------
// The loop routine runs over and over again forever:
// ----------------------------------------------------------------------------------------------------

MicroTimer displayTimer;
long long nextDisplay_ms = 1000;

void updateDisplay(void)
{
    // Display output (either LCD or Nixie)
    //if (displayTimer.read_ms() >= nextDisplay_ms)
    {
        int yr = GPS.year;
        int mon = GPS.month;
        int dy = GPS.day;
        int hr = GPS.hour;
        int min = GPS.minute;
        int sec = GPS.seconds;  // We will actually be slightly behind, but ok
        
        hr -= 7; // Tucson offset
        if (hr < 0)
        {
            hr += 24;
            dy -= 1;
        }

        nextDisplay_ms += 1000;

        // Convert time to BCD
        long x = hr;
        x = 100*x + min;
        x = 100*x + sec;

        //Display date every minute for a few seconds
        if (sec < 5)
        {
            x = yr;
            x = 100*x + mon;
            x = 100*x + dy;
        }

        long bcd = 0;
        int shift = 0;
        while (x > 0) 
        {
            bcd |= (x % 10) << (shift++ << 2);
            x /= 10;
        }
        Serial.print("BCD = ");
        Serial.println(bcd,HEX);
        updateShiftRegister(bcd >> 16, (bcd >> 8) & 0xFF, bcd & 0xFF);
    }
}

uint32_t gpstimer = millis();
bool handleGps(void)
{
    char c = GPS.read();

    // if you want to debug, this is a good time to do it!
    if ((c) && (GPSECHO))
        Serial.write(c);

    // if a sentence is received, we can check the checksum, parse it...
    if (GPS.newNMEAreceived()) 
    {
        // a tricky thing here is if we print the NMEA sentence, or data
        // we end up not listening and catching other sentences!
        // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
        //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

        if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
        {
            return false;  // we can fail to parse a sentence in which case we should just wait for another
        }
    }

    // approximately every 2 seconds or so, print out the current stats
    if (millis() - gpstimer > 1000) 
    {
        gpstimer = millis();

        Serial.print("\nTime: ");
        if (GPS.hour < 10) { Serial.print('0'); }
        Serial.print(GPS.hour, DEC); Serial.print(':');
        if (GPS.minute < 10) { Serial.print('0'); }
        Serial.print(GPS.minute, DEC); Serial.print(':');
        if (GPS.seconds < 10) { Serial.print('0'); }
        Serial.print(GPS.seconds, DEC); Serial.print('.');
        if (GPS.milliseconds < 10) {
        Serial.print("00");
        } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
        Serial.print("0");
        }
        Serial.println(GPS.milliseconds);
        Serial.print("Date: ");
        Serial.print(GPS.day, DEC); Serial.print('/');
        Serial.print(GPS.month, DEC); Serial.print("/20");
        Serial.println(GPS.year, DEC);
        Serial.print("Fix: "); Serial.print((int)GPS.fix);
        Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
        if (GPS.fix) {
        Serial.print("Location: ");
        Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
        Serial.print(", ");
        Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);

        Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
        }
        return true;
    } 

    return false;  
}

void loop()
{
    if (handleGps())
    {
        updateDisplay();
    }
}
