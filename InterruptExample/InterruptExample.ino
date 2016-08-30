// ****************************************************************************************************
//
// InterruptExample
//
// The following example demonstrates response to an interrupt input.
//
// Interrupts allow the processor to service an event without polling in the software.
// This helps produce more efficient code that can capture time critical conditions nearly
// immediately. As a consequence, the interrupt service routines (ISRs) should be as short
// as possible, usually only reading or writing some device or updating small amounts of data
// and then returning to the point of interruption.
//
// A wire from pin 13 (LED) to pin 2 (interrupt 0) will cause the attached interrupt service routine (ISR)
// to execute. The ISR changes the state variable, which is picked up by the loop and written to
// the LED again. The ISR is attached using the CHANGE mode, meaning that any change in the pin 2
// input will trigger the call to the ISR. A delay is placed in the loop so the blink can be seen.
//
// The following is a list of all of the interrupts available to the Atmega328
// The value in () is the name of the vector that can be used if setting up our own handlers
// which is only necessary if the provided libraries have not already done so.
// Most libraries take care of everything, including attaching a user-defined function to INT0
// and INT1 (pins 2 and 3, respectively)... i.e., we DON'T actually need to write our own
// vector code for invoking a user-define function on the available external interrupts.
//
//
// See http://www.gammon.com.au/forum/?id=11488 for more information on writing custom vectors (if needed)
//
// 1  Reset 
// 2  External Interrupt Request 0  (pin D2)          (INT0_vect)      attachInterrupt uses this vector
// 3  External Interrupt Request 1  (pin D3)          (INT1_vect)      attachInterrupt uses this vector
// 4  Pin Change Interrupt Request 0 (pins D8 to D13) (PCINT0_vect)
// 5  Pin Change Interrupt Request 1 (pins A0 to A5)  (PCINT1_vect)
// 6  Pin Change Interrupt Request 2 (pins D0 to D7)  (PCINT2_vect)
// 7  Watchdog Time-out Interrupt                     (WDT_vect)
// 8  Timer/Counter2 Compare Match A                  (TIMER2_COMPA_vect)
// 9  Timer/Counter2 Compare Match B                  (TIMER2_COMPB_vect)
//10  Timer/Counter2 Overflow                         (TIMER2_OVF_vect)
//11  Timer/Counter1 Capture Event                    (TIMER1_CAPT_vect)
//12  Timer/Counter1 Compare Match A                  (TIMER1_COMPA_vect)
//13  Timer/Counter1 Compare Match B                  (TIMER1_COMPB_vect)
//14  Timer/Counter1 Overflow                         (TIMER1_OVF_vect)
//15  Timer/Counter0 Compare Match A                  (TIMER0_COMPA_vect)
//16  Timer/Counter0 Compare Match B                  (TIMER0_COMPB_vect)
//17  Timer/Counter0 Overflow                         (TIMER0_OVF_vect)
//18  SPI Serial Transfer Complete                    (SPI_STC_vect)
//19  USART Rx Complete                               (USART_RX_vect)
//20  USART, Data Register Empty                      (USART_UDRE_vect)
//21  USART, Tx Complete                              (USART_TX_vect)
//22  ADC Conversion Complete                         (ADC_vect)
//23  EEPROM Ready                                    (EE_READY_vect)
//24  Analog Comparator                               (ANALOG_COMP_vect)
//25  2-wire Serial Interface  (I2C)                  (TWI_vect)
//26  Store Program Memory Ready                      (SPM_READY_vect)
//
// Here is some example code (from gammon) on how to use a LOW pulse on an interrupt pin to wake
// a power-saving (sleeping) processor.
//
//  #include <avr/sleep.h>                  
//  
//  // interrupt service routine in sleep mode
//  void wake ()
//  {
//    sleep_disable ();         // first thing after waking from sleep:
//    detachInterrupt (0);      // stop LOW interrupt
//  }  // end of wake
//  
//  void sleepNow ()
//  {
//    set_sleep_mode (SLEEP_MODE_PWR_DOWN);   
//    noInterrupts ();          // make sure we don't get interrupted before we sleep
//    sleep_enable ();          // enables the sleep bit in the mcucr register
//    attachInterrupt (0, wake, LOW);  // wake up on low level
//    interrupts ();           // interrupts allowed now, next instruction WILL be executed
//    sleep_cpu ();            // here the device is put to sleep
//  }  // end of sleepNow
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
  attachInterrupt(0, changeState, CHANGE); // calls changeState() whenever a change is detected on pin 2
  
  // NOTE: The UNO does NOT have a HIGH mode for interrupts... only LOW, CHANGE, RISING, or FALLING
}

// The loop is run from main continuously.
// The delay ensures we can visually see the execution of this logic.
//
// NOTE: This example is synchronous. I.e., the ISR is invoked in direct response to the
// digital write in the main loop. However, normally the interrupt pin would be hooked up
// to some device that produces a pulse or level condition to be recognized.
void loop()
{
  delay(delayValue);
  digitalWrite(DP_BUILT_IN_LED, state);  // Each time this call is made the blink() function will be called automatically
}

// This is the ISR.
// ISRs must have no arguments (in this Arduino environment an ISR is more like an interrupt vector in other processors)
// ISRs should always be as short as possible to allow the main thread to continue operating in a timely manner.
// Basically, get in, do only what is necessary, and get out
void changeState()
{
  delayValue = ++delayValue % 100;
  state = !state;
}
