// ****************************************************************************************************
//
// RadioBeep
//
// Plays a sound corresponding to the time inverted signal of a WWVB receiver
// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------
// Copyright (c) 2021 - Michael Kessel, Rocket RedNeck
// RocketRedNeck hereby grants license for others to copy and modify this source code for
// whatever purpose other's deem worthy as long as RocketRedNeck is given credit where
// where credit is due and you leave RocketRedNeck out of it for all other nefarious purposes.
// ****************************************************************************************************

#include "MicroTimer.h"
#include <LiquidCrystal.h>
 
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
  DP_RX            = 0,    // Arduino built in function
  DP_TX            = 1,    // Arduino built in function
  
  DP_LCD_D7       = 2,
  
  DP_LCD_D6       = 3,    // PWM
  
  DP_LCD_D5       = 4,
  
  DP_LCD_D4       = 5,    // PWM
  
  DP_WWVB          = 6,    // PWM  
  DP_WWVB_INVERTED = 7,
  DP_UNUSED8       = 8,

  DP_UNUSED9       = 9,    // PWM
  
  DP_UNUSED10      = 10,   // PWM  
  DP_LCD_EN        = 11,   // PWM  
  DP_LCD_RS        = 12,
  
  DP_BUILT_IN_LED  = 13,   // Arduino built in function
  
  DP_MAX_NUMBER_OF_PINS,
  DP_BOGUS                 // A value to recognize when none of the above is indicated.
};

// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------
enum AnalogPins
{
  AP_RANDOM_SEED  = 0,    // See notations where this is used, below
  AP_UNUSED1      = 1,
  AP_UNUSED2      = 2,
  AP_UNUSED3      = 3,
  AP_UNUSED4      = 4,
  AP_UNUSED5      = 5,
  
  AP_MAX_NUMBER_OF_PINS,
  AP_BUGUS
  
};


#define EVER (;;)
#define DIM(x)  (sizeof(x)/sizeof(x[0]))
#define LAST(x) (DIM(x)-1)

// ----------------------------------------------------------------------------------------------------
// Finally! The setup routine runs once at power-on/reset:
// ----------------------------------------------------------------------------------------------------
LiquidCrystal lcd(DP_LCD_RS, DP_LCD_EN, DP_LCD_D4, DP_LCD_D5, DP_LCD_D6, DP_LCD_D7);

void setup() 
{ 
    // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  
  pinMode(DP_BUILT_IN_LED, OUTPUT);
  pinMode(DP_WWVB, INPUT_PULLUP);
  pinMode(DP_WWVB_INVERTED, INPUT_PULLUP);
  
  lcd.println("Hello Radio");
  lcd.print("OUT = ");
  lcd.print(DP_WWVB);
  lcd.print("~OUT = ");
  lcd.print(DP_WWVB_INVERTED);
  lcd.clear();
   
}

// ----------------------------------------------------------------------------------------------------
// The loop routine runs over and over again forever:
// ----------------------------------------------------------------------------------------------------
bool on = false;
MicroTimer timer;
long long pw = 0;
void loop()
{
    // Active low output from OUT (~OUT is active high)... yeah, that is backwards, live with it.
    // The reason is the radio signal power drops 17 dB to signal the bit (at 1 Hz)
    // 200 ms dropout = 0
    // 500 ms dropout = 1
    // 800 ms dropout = mark (end of 10 second interval and first mark at minute)
    // Note: The end/start of minute is two (2) 800 ms marks in a row.
    if (digitalRead(DP_WWVB) == 0 && digitalRead(DP_WWVB_INVERTED) == 1)
    { 
      // Pulse in progress
      // Check if we reported it
      if (! on)
      {
        // reset timer on edge
        timer.mark();
        on = true;
        lcd.setCursor(0,0);
        lcd.print("ON "); 
      }
      digitalWrite(DP_BUILT_IN_LED, 1);
    }
    else if (digitalRead(DP_WWVB) == 1 && digitalRead(DP_WWVB_INVERTED) == 0)
    {
      // Idle again
      // Check if we reported it
      if (on)
      {
        // Read the timer to compute the pulse width (in microseconds)
        pw = timer.read();
        on = false;
        lcd.setCursor(0,0);
        lcd.print("OFF"); 
        lcd.setCursor(0,1);
        lcd.print("PW = "); 
        lcd.print((int)(pw/1000));
      }
      digitalWrite(DP_BUILT_IN_LED, 0);
    }   
}
