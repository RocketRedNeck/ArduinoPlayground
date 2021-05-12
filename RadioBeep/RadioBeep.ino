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
  
  DP_UNUSED2       = 2,
  
  DP_SPEAKER       = 3,    // PWM
  
  DP_UNUSED4       = 4,
  
  DP_UNUSED5       = 5,    // PWM
  
  DP_LED_RIGHT     = 6,    // PWM  
  DP_LED_CENTER    = 7,
  DP_LED_LEFT      = 8,

  DP_UNUSED9       = 9,    // PWM
  
  DP_BUTTON_RIGHT  = 10,   // PWM  
  DP_BUTTON_CENTER = 11,   // PWM  
  DP_BUTTON_LEFT   = 12,
  
  DP_BUILT_IN_LED  = 13,   // Arduino built in function
  DP_WWVB = DP_BUILT_IN_LED,
  
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

// ----------------------------------------------------------------------------------------------------
// Note frequencies in Hz
// ----------------------------------------------------------------------------------------------------
#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978

#define EVER (;;)
#define DIM(x)  (sizeof(x)/sizeof(x[0]))
#define LAST(x) (DIM(x)-1)

// ----------------------------------------------------------------------------------------------------
// Finally! The setup routine runs once at power-on/reset:
// ----------------------------------------------------------------------------------------------------
void setup() 
{  
  // Initialize the pins being used
  noTone(DP_SPEAKER);
  
  pinMode(DP_WWVB, INPUT_PULLUP);

  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
  Serial.println("Hello Radio");
  Serial.println(DP_WWVB);
   
}

// ----------------------------------------------------------------------------------------------------
// The loop routine runs over and over again forever:
// ----------------------------------------------------------------------------------------------------
bool on = false;
MicroTimer timer;
long long pw = 0;
void loop()
{
    if ( ! digitalRead(DP_WWVB))
    { 
      if (! on)
      {
        timer.reset();
        on = true;
        Serial.println("ON");
      }
      tone(DP_SPEAKER, NOTE_C2);
    }
    else
    {
      if (on)
      {
        pw = timer.read();
        on = false;
        Serial.print("off at");
        timer.println(pw/1000);
      }
      noTone(DP_SPEAKER);
    }   
}
