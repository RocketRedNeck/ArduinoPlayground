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


/*
Adafruit Arduino - Lesson 4. 8 LEDs and a Shift Register
*/

int enablePin = 2; // ~OE,    Pin 13 on 74HC595
int latchPin = 3;  // RCLK,   Pin 12 on 74HC595
int clearPin = 4;  // ~SRCLR, Pin 10 on 74HC595
int clockPin = 5;  // SRCLK,  Pin 11 on 74HC595
int dataPin = 6;   // SER,    Pin 14 on 74HC595

byte leds = 0;

void updateShiftRegister(byte msw, byte mid, byte lsw)
{
    digitalWrite(latchPin, LOW);
    shiftOut(dataPin, clockPin, MSBFIRST, msw);
    shiftOut(dataPin, clockPin, MSBFIRST, mid);
    shiftOut(dataPin, clockPin, MSBFIRST, lsw);
    digitalWrite(latchPin, HIGH);
}

void setup()
{
    // initialize GDB stub
    debug_init();

    pinMode(enablePin,  OUTPUT);
    pinMode(latchPin,   OUTPUT);
    pinMode(clearPin,   OUTPUT);
    pinMode(clockPin,   OUTPUT);
    pinMode(dataPin,    OUTPUT);

    digitalWrite(enablePin, 0);
    digitalWrite(clearPin,  1);

    for (int i = 9; i >= 0; --i)
    {
        updateShiftRegister((i << 4) | i, (i << 4) | i, (i << 4) | i);
        delay(500);
    }
}

void loop()
{
    for (long i = 0; i < 1000000; i++)
    {
        // Convert to BCD
        long input = i;
        long bcd = 0;
        int shift = 0;
        while (input > 0) 
        {
            bcd |= (input % 10) << (shift++ << 2);
            input /= 10;
        }        
        updateShiftRegister(bcd >> 16, (bcd >> 8) & 0xFF, bcd & 0xFF);
        delay(1000);
    }
}
