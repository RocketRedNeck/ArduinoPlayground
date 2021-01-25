// ****************************************************************************************************
//
// Tone Generation
//
// A musical example
// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------
// Copyright (c) 2013 - RocketRedNeck.net
// RocketRedNeck hereby grants license for others to copy and modify this source code for
// whatever purpose other's deem worthy as long as RocketRedNeck is given credit where
// where credit is due and you leave RocketRedNeck out of it for all other nefarious purposes.
// ****************************************************************************************************
 
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
// Beats per minute and note durations
// Here we choose a simple 60 BPM to keep the math easy in the head.
// Later, in the playSong function we can adjust the tempo as needed.
// ----------------------------------------------------------------------------------------------------
#define BPM          60
#define BPS          (BPM/60)
#define MSPB         (1000/BPS)

#define WHOLE        MSPB
#define HALF         (WHOLE/2)
#define QUARTER      (HALF/2)
#define EIGHTH       (QUARTER/2)
#define SIXTEENTH    (EIGHTH/2)
#define THIRTYSECOND (SIXTEENTH/2)
#define SIXTYFOURTH  (THIRTYSECOND/2)

// ----------------------------------------------------------------------------------------------------
// Some NOT so scientifically determined constants to make the game play well
// ----------------------------------------------------------------------------------------------------
#define HOLD_TIME_MSEC 450
#define GAP_TIME_MSEC  100

// ----------------------------------------------------------------------------------------------------
// Define some songs to play at various points in the game
// ----------------------------------------------------------------------------------------------------
struct NoteDescriptor
{
  long note;
  long duration;
};

// ----------------------------------------------------------------------------------------------------
// NOTE: To make pacing easier, just write the songs exactly as the notations
// in the sheet music would show (notes and such), then adjust tempo and octave
// at the playSong function as needed.
// ----------------------------------------------------------------------------------------------------
// Charge!
// ----------------------------------------------------------------------------------------------------
NoteDescriptor startSong[] = 
{
  NOTE_G3, EIGHTH,
  NOTE_C4, EIGHTH,
  NOTE_E4, EIGHTH,
  NOTE_G5, EIGHTH,    // Staccato dotted eighth sounds okay as eighth follwed by a sixteenth rest
  0,       SIXTEENTH,
  NOTE_E5, SIXTEENTH,
  NOTE_G5, HALF
};

// ----------------------------------------------------------------------------------------------------
// Chopin's Funeral March
// ----------------------------------------------------------------------------------------------------
NoteDescriptor deathMarch[] = 
{
  NOTE_B1, QUARTER,
  NOTE_B1, EIGHTH + SIXTEENTH,    // Dotted eighth note
  NOTE_B1, SIXTEENTH,
  NOTE_B1, QUARTER,
  NOTE_D2, EIGHTH + SIXTEENTH,
  NOTE_C2, SIXTEENTH,
  NOTE_C2, EIGHTH + SIXTEENTH,
  NOTE_B1, SIXTEENTH,
  NOTE_B1, EIGHTH + SIXTEENTH,
  NOTE_B1, SIXTEENTH,
  NOTE_B1, HALF
};

// ----------------------------------------------------------------------------------------------------
// When the Ants Come Marching In
// ----------------------------------------------------------------------------------------------------
NoteDescriptor antMarch[] =
{
  NOTE_E4, EIGHTH,
  NOTE_A4, QUARTER,
  NOTE_A4, EIGHTH,
  NOTE_A4, QUARTER,
  NOTE_B4, EIGHTH,
  NOTE_C5, QUARTER,
  NOTE_B4, EIGHTH,
  NOTE_C5, QUARTER,
  NOTE_A4, EIGHTH,
  NOTE_G4, HALF + EIGHTH,
  NOTE_E4, EIGHTH,
  NOTE_G4, HALF + EIGHTH
};




// ----------------------------------------------------------------------------------------------------
// softwareReset - starts the sketch over, but does not reset hardware. Must re-initialize in setup()
// ----------------------------------------------------------------------------------------------------
void softwareReset() // Restarts program from beginning but does not reset the peripherals and registers
{
   asm volatile ("jmp 0");  
}


// ----------------------------------------------------------------------------------------------------
// playSong will play an array of note/duration pairs
// The compiler was having problems pass a pointer to NoteDescriptor so we simply pass it
// as void* and cast... we have ways of making the compiler comply.
// The optional tempo scales the durations: > 1.0 is faster, < 1.0 is slow
// The optional octave scales the notes: > 1.0 is higher pitch, < 1.0 is lower pitch
// In general the octave will be multiples of 2.0**n unless you really like things
// slightly off key.
// ----------------------------------------------------------------------------------------------------
void playSong(void *song, int len, float tempo = 1.0, float octave = 1.0)
{
  NoteDescriptor *pNoteDescriptor = (NoteDescriptor *)(song);
  for (int i = 0; i < len; ++i)
  {
    if (pNoteDescriptor[i].note != 0)
    {  
       tone(DP_SPEAKER, long(pNoteDescriptor[i].note * octave));
    };
    delay(long(pNoteDescriptor[i].duration / tempo));
    noTone(DP_SPEAKER);
    delay(long(SIXTYFOURTH / tempo));
  }
}

// ----------------------------------------------------------------------------------------------------
// Finally! The setup routine runs once at power-on/reset:
// ----------------------------------------------------------------------------------------------------
void setup() 
{  
  noTone(DP_SPEAKER);
    
  playSong(startSong, DIM(startSong), 0.6, 1.0);

  delay(1000);

    // Play happy music 
  playSong(antMarch, DIM(antMarch), 1.0, 1.0);

}




void loop()
{
  
}
