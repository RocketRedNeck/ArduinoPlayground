// ****************************************************************************************************
//
// ButtonCollectorExample
//
// The following example demonstrates response to an interrupt input to collect the number of button
// pushes to be serviced. This is useful, for example, when we need to ensure that all button push
// events get serviced even if the loop() has a long exection time.
//
// The primary problem to be handled is debounce of the event. This can be handled a couple of ways.
//  1. add capacitance to the circuit, but this still does not deal with the button mechanical properties.
//  2. add a timing gate to the ISR (this example)
//
// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------
// Copyright (c) 2014 - RocketRedNeck.net
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
  
  DP_UNUSED2       = 2,    // Also Interrupt 0
  
  DP_UNUSED3       = 3,    // PWM, also Interrupt 1
  
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

// ----------------------------------------------------------------------------------------------------
// Useful functions before defining to the setup() and loop() functions.
// ----------------------------------------------------------------------------------------------------

// ----------------------------------------------------------------------------------------------------
// softwareReset - starts the sketch over, but does not reset hardware. Must re-initialize in setup()
// ----------------------------------------------------------------------------------------------------
void softwareReset() // Restarts program from beginning but does not reset the peripherals and registers
{
   asm volatile ("jmp 0");  
}

// Variables modifed by ISRs should always be defined as volatile to ensure
// that the access is never optimized away in loops
volatile int state = LOW;    // Starting LOW because we know the LED (pin 13) is driven high by pinMode(...)
volatile int delayValue = 0; // Will be modified in a moduo sense up to 100 to create a pulsating (laughing) effect

// In setup we define the LED output state and then attach the blink() function
// to interrupt 0 (which corresponds to pin 2).
void setup()
{
  pinMode(DP_BUILT_IN_LED, OUTPUT);  // LED is driven HIGH at initialization
  attachInterrupt(0, eventDetection, FALLING); // calls eventDetection() whenever a falling edge is detected on pin 2
  Serial.begin(115200);
  
  // NOTE: The UNO does NOT have a HIGH mode for interrupts... only LOW, CHANGE, RISING, or FALLING
}

// The loop is run from main continuously.
// The delay ensures we can visually see the execution of this logic.
//
// NOTE: This example is asynchronous; i.e., the ISR is invoked whenever pin 2 falls to ground.
// The number of times the button attached to pin 2 is pressed is counted using a lossless counter.
// The separate write/read counters allows the accumulation (virtually clearing the count) to occur
// without needing to disable the interrupts in either a producer or consumer context. Data queues
// can operate in a similar fashion by using the event counters as an index into a buffer space
// taking care of overflow issues with simple limit checks.
long readCounter = 0;
long writeCounter = 0;
void loop()
{
  delay(1000);
  long numberOfEvents = (writeCounter - readCounter);
  readCounter += numberOfEvents;

  Serial.print("Number of Events = ");
  Serial.println(numberOfEvents);
}

// This is the ISR.
// ISRs must have no arguments (in this Arduino environment an ISR is more like an interrupt vector in other processors)
// ISRs should always be as short as possible to allow the main thread to continue operating in a timely manner.
// Basically, get in, do only what is necessary, and get out
// All of the low-level interrupt handling (acknowledgements and/or reset conditions required by the hardware) are
// already built into the INT0 or INT1 vector handling provided with the built-in library function, attachInterrupt().
unsigned long lastDetectionTime = 0;
unsigned long currentDetectionTime = 0;
void eventDetection()
{
  currentDetectionTime = millis();
  if ((currentDetectionTime - lastDetectionTime) > 100) // In this case we limit the registration to events at least 100 ms apart.
  {                                                     // In fact, this is a typical requirement for edge sensitive logic, protecting
    // This looks like a legitimate button press        // the software from noise or bouncy inputs. Of course, the circuit could also
    lastDetectionTime = currentDetectionTime;           // use filters (capcitors) to limit the bounce.
    ++writeCounter;
  }
}
