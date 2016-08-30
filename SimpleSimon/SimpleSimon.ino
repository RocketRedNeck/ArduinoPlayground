// ****************************************************************************************************
//
// SimpleSimon
//
// A 3-state game of Simon that flashes LEDs in a progressively longer sequence
// that the user must copy (via button pushes) until the maximum sequence has been reached. 
// To augment the game a tone is played with each position displayed (both Simon and user).
//
// Any error ends the game with a sad tune, a playback of the sequence, and then an alternating 
// Left-Right then Center blinking pattern.
//
// Reaching the maximum sequence will play a happy tune, pattern playback, then blink all three LEDs.
//
// The game is resettable after winning or losing by pressing and holding any button during or after
// pattern playback.
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
// Set up an array to store pattern.
// To minimize translation of left/center/right we just create an array of the pin assignments, even
// though we will only be using three of the values. The mapping of button/pin/index is handle via
// a couple of simple mapping arrays because RocketRedNeck hates maintaining switch statements
// ----------------------------------------------------------------------------------------------------
const int MAX_PATTERN_DEPTH = 100;
int CurrentIndex;                          // Start at the begining, but be initialized at setup()
DigitalPins Pattern[MAX_PATTERN_DEPTH];    // Will be initialized in setup()

// ----------------------------------------------------------------------------------------------------
// The following index mappings will make translation, at runtime, simple and faster, at the
// expense of some memory.
// The mappings are initialized at startup such that that any unmapped inputs and outputs will be
// default to the LEFT LED/BUTTON position (index 0)
// ----------------------------------------------------------------------------------------------------
enum Positions
{
  POS_LEFT   = 0,
  POS_CENTER = 1,
  POS_RIGHT  = 2,
  
  POS_MAX_NUMBER_OF_POSITIONS
  
};

Positions LEDToPosition[DP_MAX_NUMBER_OF_PINS];
Positions BUTTONToPosition[DP_MAX_NUMBER_OF_PINS];

DigitalPins PositionToLED[POS_MAX_NUMBER_OF_POSITIONS];
DigitalPins PositionToBUTTON[POS_MAX_NUMBER_OF_POSITIONS];

int PositionToNote[POS_MAX_NUMBER_OF_POSITIONS];

// ----------------------------------------------------------------------------------------------------
// class for maintaining a debounced button state
//
// While many variations of the debounce exist, this one works for this application nicely.
// Ideally it should be a template, but RocketRedNeck has not experimented with the underlying
// Arduino compiler enough to know if it would be a problem... later perhaps.
// ----------------------------------------------------------------------------------------------------

class Button
{
  protected:
    int  pin_;
    int  currentState_;
    int  lastState_;
    long lastButtonTime_msec_;
    long debounceTime_msec_;
    
  public:
    // ----------------------------------------------------------------------------------------------------
    // ----------------------------------------------------------------------------------------------------
    Button(int digitalInputPin, long debounceTime_msec = 50)
    {
      pin_ = digitalInputPin;
      debounceTime_msec_ = debounceTime_msec;

      reset();      
    }
    
    // ----------------------------------------------------------------------------------------------------
    // ----------------------------------------------------------------------------------------------------
    void reset()
    {
      // Assume that the button is NOT pressed but record the time "now"
      currentState_ = LOW;
      lastState_    = LOW;
      lastButtonTime_msec_ = millis();
      
    }
    
    // ----------------------------------------------------------------------------------------------------
    // ----------------------------------------------------------------------------------------------------
    int getState()
    {
      int thisState = digitalRead(pin_);
      if (thisState != lastState_)
      {
        lastButtonTime_msec_ = millis();
      }
      
      if ((millis() - lastButtonTime_msec_) > debounceTime_msec_) 
      {
         if (thisState != currentState_) 
         {
            currentState_ = thisState;
          }
      }
      
      lastState_ = thisState;

      return currentState_;      
    }
    
    // ----------------------------------------------------------------------------------------------------
    // ----------------------------------------------------------------------------------------------------
    bool isOn()
    {
      return getState() == HIGH;
    }
    
    // ----------------------------------------------------------------------------------------------------
    // ----------------------------------------------------------------------------------------------------
    bool isOff()
    {
      return getState() == LOW;
    }
};

// ----------------------------------------------------------------------------------------------------
// Now create the Button instances using the default debounce timing
// ----------------------------------------------------------------------------------------------------
Button ButtonLeft(DP_BUTTON_LEFT);
Button ButtonCenter(DP_BUTTON_CENTER);
Button ButtonRight(DP_BUTTON_RIGHT);

// ----------------------------------------------------------------------------------------------------
// softwareReset - starts the sketch over, but does not reset hardware. Must re-initialize in setup()
// ----------------------------------------------------------------------------------------------------
void softwareReset() // Restarts program from beginning but does not reset the peripherals and registers
{
   asm volatile ("jmp 0");  
}

// ----------------------------------------------------------------------------------------------------
// resetOnAnyButton - initiates a softwareReset if any of our buttons are currently ON
// ----------------------------------------------------------------------------------------------------
void resetOnAnyButton()
{
  if (ButtonLeft.isOn() || ButtonCenter.isOn() || ButtonRight.isOn())
  {
    softwareReset();
  }
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
       digitalWrite(PositionToLED[pNoteDescriptor[i].note % POS_MAX_NUMBER_OF_POSITIONS], HIGH);
    };
    delay(long(pNoteDescriptor[i].duration / tempo));
    noTone(DP_SPEAKER);
    digitalWrite(PositionToLED[pNoteDescriptor[i].note % POS_MAX_NUMBER_OF_POSITIONS], LOW);
    delay(long(SIXTYFOURTH / tempo));
  }
}

// ----------------------------------------------------------------------------------------------------
// Finally! The setup routine runs once at power-on/reset:
// ----------------------------------------------------------------------------------------------------
void setup() 
{  
  // Initialize the pins being used
  pinMode(DP_LED_LEFT,   OUTPUT);     
  pinMode(DP_LED_CENTER, OUTPUT);
  pinMode(DP_LED_RIGHT,  OUTPUT);
  pinMode(DP_SPEAKER,    OUTPUT);
  
  digitalWrite(DP_LED_LEFT,   LOW);
  digitalWrite(DP_LED_CENTER, LOW);
  digitalWrite(DP_LED_RIGHT,  LOW);
  noTone(DP_SPEAKER);
  
  pinMode(DP_BUTTON_LEFT,   INPUT);
  pinMode(DP_BUTTON_CENTER, INPUT);
  pinMode(DP_BUTTON_RIGHT,  INPUT);
  
  ButtonLeft.reset();
  ButtonCenter.reset();
  ButtonRight.reset();
  
  // Create a pin index map, in both directions.
  PositionToLED[POS_LEFT]   = DP_LED_LEFT;
  PositionToLED[POS_CENTER] = DP_LED_CENTER;
  PositionToLED[POS_RIGHT]  = DP_LED_RIGHT;
  
  PositionToBUTTON[POS_LEFT]   = DP_BUTTON_LEFT;
  PositionToBUTTON[POS_CENTER] = DP_BUTTON_CENTER;
  PositionToBUTTON[POS_RIGHT]  = DP_BUTTON_RIGHT;
  
  PositionToNote[POS_LEFT]     = NOTE_C5;
  PositionToNote[POS_CENTER]   = NOTE_E5;
  PositionToNote[POS_RIGHT]    = NOTE_G5;

  // We don't need to type in the inverse
  // Simply loop through the map to create the identity relationships
  for (int pos = POS_LEFT; pos < POS_MAX_NUMBER_OF_POSITIONS; ++pos)
  {
    LEDToPosition[PositionToLED[pos]] = (Positions)pos;
    BUTTONToPosition[PositionToBUTTON[pos]] = (Positions)pos;
  }  
  
  // Randomly assign either left/center/right
  // Read an analog input to seed the random seed just
  // to keep things interesting; of course this does not
  // work if something is connected to the pin in a manner
  // that causes a constant to be used. Ideally some form
  // of wide noise generation would be put on the pin to
  // give a really random experience.
  int seedRead = analogRead(AP_RANDOM_SEED);
  randomSeed(seedRead);
  
  for (int i = 0; i < MAX_PATTERN_DEPTH; ++i)
  {
    Pattern[i] = PositionToLED[random(POS_LEFT, POS_MAX_NUMBER_OF_POSITIONS)];
  }
  
  // Start at the beginning
  CurrentIndex = 1;
  
  playSong(startSong, DIM(startSong), 0.6, 1.0);
  
  delay(2000);
   
}

// ----------------------------------------------------------------------------------------------------
// Other helper functions before getting to the main loop
// ----------------------------------------------------------------------------------------------------

// ----------------------------------------------------------------------------------------------------
// playPattern - runs through the pattern at the pace indicated
// ----------------------------------------------------------------------------------------------------
void playPattern(int aDepth, int aHoldTime_msec, int aGapTime_msec, bool checkForExit = false)
{
  // Play the entire pattern, quickly
  // Loop through the pattern 1 step at a time, waiting for user to enter a button push
  for (int i = 0; i < aDepth; ++i)
  {
    // If any button is pressed, start the sketch over
    if (checkForExit)
    {
      resetOnAnyButton();
    }
    
   // Light up the LED and play a sound
    digitalWrite(Pattern[i],HIGH);
    tone(DP_SPEAKER, PositionToNote[LEDToPosition[Pattern[i]]]);
    
    // Wait for it!
    // NOTE: Using the delay here is simpler than letting tone()
    // do it with the hardware; much less timing issues.
    delay(aHoldTime_msec);
    
    // Turn LED and sound off
    noTone(DP_SPEAKER);
    digitalWrite(Pattern[i],LOW);
    
    // Wait a little before going to next,
    // it just looks and sounds better
    delay(aGapTime_msec);
  }
}

// ----------------------------------------------------------------------------------------------------
// gameOver - user loses, sad music is played and then we just wait for a reset
// ----------------------------------------------------------------------------------------------------
void gameOver()
{
  // Play sad music
  playSong(deathMarch, DIM(deathMarch), 0.25, 2.0);
  
  // Play the entire pattern, quickly, while checking for user key press to
  // reset game imediately, if desired.
  playPattern(MAX_PATTERN_DEPTH, HOLD_TIME_MSEC/2, GAP_TIME_MSEC/2, true);
  
  // Blink 101, 010, ... slowly
  for EVER
  {
    digitalWrite(DP_LED_LEFT,   HIGH);
    digitalWrite(DP_LED_CENTER, LOW);
    digitalWrite(DP_LED_RIGHT,  HIGH);
    delay(500);
    
    // If any button is pressed, start the sketch over
    resetOnAnyButton();
    
    digitalWrite(DP_LED_LEFT,   LOW);
    digitalWrite(DP_LED_CENTER, HIGH);
    digitalWrite(DP_LED_RIGHT,  LOW);
    delay(500);
    
    // If any button is pressed, start the sketch over
    resetOnAnyButton();
    
  }
}

// ----------------------------------------------------------------------------------------------------
// youWin - user entered the entire pattern correctly, play happy tune and then wait for a reset
// ----------------------------------------------------------------------------------------------------
void youWin()
{
  // Play happy music 
  playSong(antMarch, DIM(antMarch), 1.0, 1.0);

  // Play the entire pattern, quickly, while checking for user key press to
  // reset game imediately, if desired.
  playPattern(MAX_PATTERN_DEPTH, HOLD_TIME_MSEC/2, GAP_TIME_MSEC/2, true);  
  
  // Blink 111, 000, ... rapidly
  for EVER
  {
    digitalWrite(DP_LED_LEFT,   HIGH);
    digitalWrite(DP_LED_CENTER, HIGH);
    digitalWrite(DP_LED_RIGHT,  HIGH);
    delay(100);
    
    // If any button is pressed, start the sketch over
    resetOnAnyButton();
    
    digitalWrite(DP_LED_LEFT,   LOW);
    digitalWrite(DP_LED_CENTER, LOW);
    digitalWrite(DP_LED_RIGHT,  LOW);
    delay(100);
    
    // If any button is pressed, start the sketch over
    resetOnAnyButton();

  }
}

// ----------------------------------------------------------------------------------------------------
// The loop routine runs over and over again forever:
// ----------------------------------------------------------------------------------------------------

void loop()
{
  // Loop through the pattern 1 step at a time, waiting for user to enter a button push
  playPattern(CurrentIndex, HOLD_TIME_MSEC, GAP_TIME_MSEC);
    
  // Now loop through the user's input but if they wait more than
  // 5 seconds between any button push, or hold a button for more than
  // 5 seconds, the game ends (unless it is the last button push for
  // the sequence, in which case we stop accepting button pushes and
  // go back to the pattern).
  for (int i = 0; i < CurrentIndex; ++i)
  {
    // Wait up to 5 seconds for a button push, debounced of course
    long start_msec = millis();
    int thisButton = -1;  // Until proven otherwise
    ButtonLeft.reset();
    ButtonCenter.reset();
    ButtonRight.reset();
    while ((millis() - start_msec) < 5000)
    {
      // Find one or more buttons that are ON (after debouce)
      // Since the isOn() function implies a reading of the button
      // state, we read it once per timing loop, here, to avoid
      // or at least minimize race conditions on a state change.
      bool buttonLeft   = ButtonLeft.isOn();
      bool buttonCenter = ButtonCenter.isOn();
      bool buttonRight  = ButtonRight.isOn();
      
      // Only exit the loop with the button number on an exclusive hit (i.e., one button only)
      if (buttonLeft ^ buttonCenter ^ buttonRight)
      {
        // Only one button is on at this time, at least it has been for the
        // minimum debounce time, so we will accept that button as the first
        // choice.
        if (buttonLeft)
        {
          thisButton = POS_LEFT;
        }
        else if (buttonCenter)
        {
          thisButton = POS_CENTER;
        }
        else
        {
          thisButton = POS_RIGHT;
        }
        break;
      }
    }
        
    // If the above loop exits without a choice, then we are done
    if (thisButton == -1)
    {
      gameOver();
    }
    
    // Now that a button choice has been made, we need to light up the
    // corresponding LED and play the note until the user lets go of the button,
    // but we will time out at 5 seconds and end the game if the user does
    // not stop holding the button
    digitalWrite(PositionToLED[thisButton], HIGH);
    tone(DP_SPEAKER, PositionToNote[thisButton]);    
    
    start_msec = millis();
    bool buttonIsOff = false;
    while ((millis() - start_msec) < 5000)
    {
      // Only exit if the time has expired or all buttons are off      
      if (ButtonLeft.isOff() && ButtonCenter.isOff() && ButtonRight.isOff())
      {
        buttonIsOff = true;
        break;
      }
    }    
    
    // For one reason or another it is time to turn off the LED and sound
    digitalWrite(PositionToLED[thisButton], LOW);
    noTone(DP_SPEAKER);    
    
    // If we get here and the button is not off the game is over
    if (! buttonIsOff)
    {
      gameOver();
    }
    
    // Now we need to check the answer
    // First wrong button push and the game is over
    if (thisButton != LEDToPosition[Pattern[i]])
    {
      gameOver();
    }
    
  }
  
  // Must have been the right answer, delay a small amount
  // and continue
  delay(2 * HOLD_TIME_MSEC);
    
  // If we exit the user loop, above, advance the index, unless
  // it is the last one, in which case the user won.
  ++CurrentIndex;
  if (CurrentIndex > MAX_PATTERN_DEPTH)
  {
    youWin();
  }
  
}
