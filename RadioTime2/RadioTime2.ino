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

#include <LiquidCrystal.h>
#include "MicroTimer.h"
 
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

int8_t data[60];
uint8_t index = 0;

void setup() 
{ 
    // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  lcd.clear();
  
  pinMode(DP_BUILT_IN_LED, OUTPUT);
  pinMode(DP_WWVB, INPUT_PULLUP);
  pinMode(DP_WWVB_INVERTED, INPUT_PULLUP);
  
  lcd.println("Hello Radio");
  lcd.print("OUT = ");
  lcd.print(DP_WWVB);
  lcd.print("~OUT = ");
  lcd.print(DP_WWVB_INVERTED);
   
}

// ----------------------------------------------------------------------------------------------------
// The loop routine runs over and over again forever:
// ----------------------------------------------------------------------------------------------------
bool on = false;
unsigned long pw_ms = 0;
enum State
{
  WAITING_FOR_END,
  WAITING_FOR_START,
  WAITING_FOR_BIT,
  WAITING_FOR_MARK
};
State state = WAITING_FOR_END;

MicroTimer timer;
long long topOfMinute_ms = 0;
long long nextBitWindow_ms = 0;
long long nextMarkWindow_ms = 0;
long long nextMinuteWindow_ms = 0;

const unsigned long MARK_LOW_MS   = 700;
const unsigned long MARK_DELTA_MS = 9000;

const unsigned long BIT_0_LOW_MS  = 100;
const unsigned long BIT_0_HIGH_MS = 300;
const unsigned long BIT_1_LOW_MS  = 400;
const unsigned long BIT_1_HIGH_MS = 600;

bool decodeTime = false;
unsigned int minutes = 0;
unsigned int hours = 0;
unsigned int days = 0;
unsigned int year = 0;

void loop()
{
    // Active low output from OUT (~OUT is active high)... yeah, that is backwards, live with it.
    // The reason is the radio signal power drops 17 dB to signal the bit (at 1 Hz)
    // 200 ms dropout = 0
    // 500 ms dropout = 1
    // 800 ms dropout = mark (end of 10 second interval and first mark at minute)
    // Note: The end/start of minute is two (2) 800 ms marks in a row.
    // If no pulse within 950 ms, timeout and try again
    pw_ms = pulseInLong(DP_WWVB_INVERTED, HIGH, 2000000) / 1000;

    if (pw_ms)
    {
      switch(state)
      {
        case WAITING_FOR_END:
          if (pw_ms > MARK_LOW_MS)
          {
            state = WAITING_FOR_START;

            // Decode time and display
            decodeTime = true;
          }
          break;
        case WAITING_FOR_START:
          if (pw_ms > MARK_LOW_MS)
          {
            topOfMinute_ms = timer.read_ms() - pw_ms; 
            nextMinuteWindow_ms = topOfMinute_ms + 60000;
            nextMarkWindow_ms   = topOfMinute_ms +  9000;
            nextBitWindow_ms    = topOfMinute_ms +  1000;
            state = WAITING_FOR_BIT;
            index = 0;
            data[index] = 1;
            ++index;
          }
          else
          {
            state = WAITING_FOR_END;
          }
          break;
        case WAITING_FOR_BIT:
          if (timer.read_ms() >= nextBitWindow_ms)
          {
            nextBitWindow_ms += 1000;
            if ( (pw_ms > BIT_1_LOW_MS) && (pw_ms < BIT_1_HIGH_MS) )
            {
              // Record 1
              data[index] = 1;
            }
            else if ( (pw_ms > BIT_0_LOW_MS) && (pw_ms < BIT_0_HIGH_MS))
            {
              // Record 0
              data[index] = 0;
            }
            else
            {
              // Invalid bit
              data[index] = -1;
            }
            ++index;
            switch (index)
            {
              case 9:
              case 19:
              case 29:
              case 39:
              case 49:
                state = WAITING_FOR_MARK;
                break;
              case 59:
                state = WAITING_FOR_END;
                break;
              default:
                break;
            }
          }
          break;
        case WAITING_FOR_MARK:
          if (timer.read_ms() >= nextMarkWindow_ms)
          {
            if (pw_ms > MARK_LOW_MS)
            {
              data[index] = 1;
            }
            else
            {
              // Invalid mark
              data[index] = -1;
            }
            ++index;
            nextMarkWindow_ms += 10000;
            nextBitWindow_ms  +=  1000;
            state = WAITING_FOR_BIT;
          }
          break;

        default:
          state = WAITING_FOR_END;
      };

      if (decodeTime)
      {
        // TODO
        minutes = 0;
        minutes += (data[1]?40:0);
        minutes += (data[2]?20:0);
        minutes += (data[3]?10:0);
        minutes += (data[5]?8:0);
        minutes += (data[6]?4:0);
        minutes += (data[7]?2:0);
        minutes += (data[8]?1:0);

        hours = 0;
        hours += (data[12]?20:0);
        hours += (data[13]?10:0);
        hours += (data[15]?8:0);
        hours += (data[16]?4:0);
        hours += (data[17]?2:0);
        hours += (data[18]?1:0);
        
        
        decodeTime = false;
      }

      // Display output (either LCD or Nixie)
      lcd.clear();
      lcd.home();
      lcd.print("PW = "); 
      lcd.print(pw_ms);
      lcd.print(" B = ");
      lcd.print(data[(index == 0)?0:(index-1) % 60]);
      lcd.setCursor(0,1);
      lcd.print("S=");
      lcd.print(state);
      lcd.print(" ");
      lcd.print(hours);
      lcd.print(":");
      lcd.print(minutes);
      lcd.print(":");
      lcd.print(index-1);      
    }
    else
    {
      lcd.clear();
      lcd.home();
      lcd.print("NO PULSE"); 
      lcd.print(" S = ");
      lcd.print(state);
    }  
}
