#include "NeopixelDisplayCommands.h"
#ifdef __AVR__
  #include <avr/power.h>
#endif

const uint16_t PIN  = 6;
const uint16_t PIXELS = 16;
//const uint16_t MAX_BRIGHTNESS = 32; //  1/8 max brightness, (mostly) eye safe
//const uint16_t MAX_BRIGHTNESS = 255; //  100% brightness, NOT eye safe (AND VERY WARM)
const uint16_t MAX_BRIGHTNESS = int(255/4);

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags (OPT.), add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
// Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXELS, PIN, NEO_GRB + NEO_KHZ800);

// Example of creating an automatically sized array of objects
// The constructors are called at program elaboration and each
// object created is COPIED into each location of the array
//
// Use something like sizeof(myStrips)/sizeof(myStrips[0]) to find
// the dimension of the array
//
// These objects are created in a memory space known as "bss" or
// base static section... usually just referred to as "static" memory
// Static memory is where all initialized objects are stored
//
// int x = 1; is stored in static memory and initialized to 1
// but unlike "objects" it has an explicit initialization and
// being "plain old data" (POD) will be stored in a section
// of memory called "data"
// where
// int x; is also static memory, it is stored with the
// other non-POD objects (bss), but will be initialized
// to zero (0) based on the C99 standard.
//
// This distinction will be important when we discuss "heap"
// memory later (which does NOT initialize memory)

Adafruit_NeoPixel myStrips[] =
{
  Adafruit_NeoPixel(PIXELS, PIN, NEO_GRB + NEO_KHZ800)
};

// Run-time strip allocation is not required...

void setup()
{
  myStrips[0].begin();
  myStrips[0].setBrightness(MAX_BRIGHTNESS);
  fill(myStrips[0], BLACK); // Initialize all pixels to 'off'
}

void loop()
{
//  fill(myStrips[0], YELLOW);
  delay(20);
//  fill(myStrips[0], MAGENTA);
  delay(20);
  fill(myStrips[0], WHITE);
}


