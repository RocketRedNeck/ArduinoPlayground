// ****************************************************************************************************
//
// DebouncingDriver
//
// The following is an experiment in applying useful C++ features within the Arduino programming
// environment; in particular, this experiment depmonstrates the use of template classes to handle
// problems in both static and dynamic polymorphic behaviors by providing a machanism to bind
// functional actions within the debounce query for a button, but do so in a manner that does
// no depend upon delays.
//
// The reader is cautioned, strongly, to only consider polymorphism only when it is absolutely necessary
// as the application of such (either statically through template typing, or dynamically through
// virtual methods) can make the code more difficult to analyze and debug.
//
// However, the purpose of this experiment is to see how far we can push the Arduino enviornment
// in the area maintainable code practices.
//
// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------
// Copyright (c) 2013 - RocketRedNeck.net
// RocketRedNeck hereby grants license for others to copy and modify this source code for
// whatever purpose other's deem worthy as long as RocketRedNeck is given credit where
// where credit is due and you leave RocketRedNeck out of it for all other nefarious purposes.
// ****************************************************************************************************

// ----------------------------------------------------------------------------------------------------
// Useful macros
// ----------------------------------------------------------------------------------------------------

#define EVER (;;)
#define DIM(x)  (sizeof(x)/sizeof(x[0]))
#define LAST(x) (DIM(x)-1)

// ----------------------------------------------------------------------------------------------------
// Some constants are herein #defined or enumerated rather than instantiated as const to avoid using 
// memory unless the values are actually used in the code below. Mileage depends on quality of linker
// and rather than tempt fate we just revert to old habits.
// ----------------------------------------------------------------------------------------------------

// ----------------------------------------------------------------------------------------------------
// Pin assignments - enumeration simply to group them nicely
// NOTE: built-in and PWMs can be allocated for any purpose, and as such the comments simply keep
// thing visible to readers so we understand any potential conflicts. Read the specs to see what
// other functions overlap
// ----------------------------------------------------------------------------------------------------
enum DigitalPins
{
  DP_RX            = 0,    // Arduino built in function
  DP_TX            = 1,    // Arduino built in function
  
  DP_UNUSED2       = 2,
  
  DP_UNUSED3       = 3,    // PWM
  
  DP_UNUSED4       = 4,
  
  DP_UNUSED5       = 5,    // PWM
  
  DP_UNSUED6       = 6,    // PWM  
  DP_UNUSED7       = 7,
  DP_UNUSED8       = 8,

  DP_UNUSED9       = 9,    // PWM
  
  DP_UNUSED10      = 10,   // PWM  
  DP_UNUSED11      = 11,   // PWM  
  DP_UNUSED12      = 12,
  
  DP_BUILT_IN_LED  = 13,   // Arduino built in function
  
  DP_MAX_NUMBER_OF_PINS,
  DP_BOGUS                 // A value to recognize when none of the above is indicated.
};

// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------
enum AnalogPins
{
  AP_UNUSED0      = 0,
  AP_UNUSED1      = 1,
  AP_UNUSED2      = 2,
  AP_UNUSED3      = 3,
  AP_UNUSED4      = 4,
  AP_UNUSED5      = 5,
  
  AP_MAX_NUMBER_OF_PINS,
  AP_BUGUS
  
};

class MicroTimer
{
  public:
    // ----------------------------------------------------------------------------------------------------
    // Constructor - implements vitual reset in a manner that is efficient at construction, yet transparent
    // in syntax.
    // ----------------------------------------------------------------------------------------------------
    MicroTimer()
    {
      reset(true);
    }
    
    // ----------------------------------------------------------------------------------------------------
    // reset - transparent interface for both constrution and general reset conditions
    // ----------------------------------------------------------------------------------------------------
    void reset(bool isConstructor = false)
    {
      if (isConstructor)
      {
        upperTime_ = 0;
        lowerTime_ = 0;
        lastLowerTime_ = 0;
      }
      
      unmark();
    }

    // ----------------------------------------------------------------------------------------------------
    // mark - makes the current absolute time the local zero such that read() is in terms of the mark.
    // ----------------------------------------------------------------------------------------------------
    void mark()
    {
      unmark();
      markTime_ = read();
    }

    // ----------------------------------------------------------------------------------------------------
    // unmark - removes any previous mark such that the read() is in terms of absolute time.
    // ----------------------------------------------------------------------------------------------------
    void unmark()
    {
      markTime_ = 0;
    }

    // ----------------------------------------------------------------------------------------------------
    // read - reads the desired time base, compensating for rollover and any marked times.
    //
    // The function returns signed values to allow simplified arithmetic in measuring time deltas for
    // which a negative time is allowable.
    //
    // NOTE: This read takes about 36 usec on an Uno Rev 3, but has the advantage that it manages the
    // rollover, thus we have a timer precise to approximately 0.036 msec but will not roll over for
    // a really long time (i.e., > 500,000 years), which is way better than every 70 minutes, with
    // sub-millisecond precision. A good compromise.
    // ----------------------------------------------------------------------------------------------------
    long long read()
    {
      lastLowerTime_ = lowerTime_;
      lowerTime_ = micros();
      if (lowerTime_ < lastLowerTime_)
      {
        ++upperTime_;
      }
      
      time_ = lowerTime_;
      time_ += ((long long)upperTime_)<<32;
      return time_ - markTime_;
    }
  
    // ----------------------------------------------------------------------------------------------------
    // print - prints time to the serial interface.
    // ----------------------------------------------------------------------------------------------------
    void print(const long long &aTime)
    {
      Serial.println((double)aTime/1.0e6,6);
    }
    void print()
    {
      print(read());
    }
  
  protected:
    unsigned long upperTime_;
    unsigned long lowerTime_;
    long long time_;
    unsigned long lastLowerTime_;
    long long markTime_;
    
  private:
};

// ----------------------------------------------------------------------------------------------------
// class for maintaining a debounced state on any digital or analog input
// ----------------------------------------------------------------------------------------------------
template <int (*PIN_FUNCTION)(int), 
          int PIN,
          long DEBOUNCE_TIME,
          typename MicroTimer_TYPE,
          int THRESHOLD = HIGH>
class Debouncer
{
  public:
    // ----------------------------------------------------------------------------------------------------
    // Constructor - implements vitual reset in a manner that is efficient at construction, yet transparent
    // in syntax.
    // ----------------------------------------------------------------------------------------------------
    Debouncer()
    {
      reset(true);
    }
    
    // ----------------------------------------------------------------------------------------------------
    // reset - transparent interface for both construction and general reset conditions
    // ----------------------------------------------------------------------------------------------------
    virtual void reset(bool isConstructor = false)
    {
      currentState_ = PIN_FUNCTION(PIN);
      lastStateTime_ = MicroTimer_.read();
      
      lastState_ = currentState_;
    }

    // ----------------------------------------------------------------------------------------------------
    // read - reads the current debounced input state
    // ----------------------------------------------------------------------------------------------------
    int read()
    {
      long long nowTime = MicroTimer_.read();
      
      int thisState = PIN_FUNCTION(PIN);
      if (abs(thisState - lastState_) >= THRESHOLD)
      {
         lastStateTime_ = nowTime;
      }
      
      if ((nowTime - lastStateTime_) > DEBOUNCE_TIME) 
      {
         if (abs(thisState - currentState_) >= THRESHOLD) 
         {
            currentState_ = thisState;
          }
      }
      
      lastState_ = thisState;

      return currentState_;   
    }  
    
    // ----------------------------------------------------------------------------------------------------
    // ----------------------------------------------------------------------------------------------------
    bool isActive()
    {
      return read() >= THRESHOLD;
    }
    
    // ----------------------------------------------------------------------------------------------------
    // ----------------------------------------------------------------------------------------------------
    bool isInactive()
    {
      return read() < THRESHOLD;
    }
  protected:
    int currentState_;
    int lastState_;
    long long lastStateTime_;
    MicroTimer_TYPE MicroTimer_;
  private:
};

// ----------------------------------------------------------------------------------------------------
// Useful functions before defining to the setup() and loop() functions.
// ----------------------------------------------------------------------------------------------------

// ----------------------------------------------------------------------------------------------------
// softwareReset - starts the sketch over, but does not reset hardware. Must re-initialize in setup()
// ----------------------------------------------------------------------------------------------------
unsigned long softwareResetTime_usec = 0;
void softwareReset() // Restarts program from beginning but does not reset the peripherals and registers
{
   softwareResetTime_usec = micros();
   asm volatile ("jmp 0");  
}

// ----------------------------------------------------------------------------------------------------
// setup - first thing to run before the infinite loop()
// ----------------------------------------------------------------------------------------------------
MicroTimer theMicroTimer;
long long setupTimeStart_usec;
long long setupTimeEnd_usec;
void setup()
{
  setupTimeStart_usec = theMicroTimer.read(); 
  Serial.begin(9600);
  setupTimeEnd_usec = theMicroTimer.read();
  
  theMicroTimer.mark();
  for (int i = 0; i < 10000; ++i)
  {
    theMicroTimer.read();
  }
  theMicroTimer.print();
  
}

// ----------------------------------------------------------------------------------------------------
// loop - repeat forever as fast as it can
// ----------------------------------------------------------------------------------------------------
void loop()
{
//  delay(1000);
  //theMicroTimer.print();
}
