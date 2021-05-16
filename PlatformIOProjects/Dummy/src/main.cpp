#include "Arduino.h"
#include "avr8-stub.h"  // Must be included to enable debug. Must add avr-debugger library

// See this for debug notes: https://docs.platformio.org/en/latest/plus/debug-tools/avr-stub.html

void setup()
{
  // initialize GDB stub
  debug_init();
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
}