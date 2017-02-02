#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#define PIN 6

const unsigned int PIXELS = 16;
const unsigned int MAX_BRIGHTNESS = 32; //  1/8 max brightness, (mostly) eye safe

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags (OPT.), add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXELS, PIN, NEO_GRB + NEO_KHZ800);

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
//
Adafruit_NeoPixel myStrips[] =
{
  Adafruit_NeoPixel(PIXELS, PIN, NEO_GRB + NEO_KHZ800),
  Adafruit_NeoPixel(PIXELS, PIN, NEO_GRB + NEO_KHZ800),
  Adafruit_NeoPixel(PIXELS, PIN, NEO_GRB + NEO_KHZ800)
}

// Example of run-time construction of an array of objects
// We initialize a pointer to NULL so we know what state
// it is in when we start; in this case we can use NULL to
// determine if the pointer is valid (although that is
// still a weak test for generally secure code)
//
// While, technically, a static pointer is alway consider
// POD (i.e., it is a pointer to an object, not an object
// itself) and would be initialized to 0 based on the C99
// rules for initializing bss memory, it is considered
// a best practice to explicitly initialize pointers to NULL
// so we get into the habit of doing so in all parts of
// our code.
Adafruit_NeoPixel *pMyStrips = NULL;

// Here we will explicitly define a constant to
// represent some number of dynamically created
// objects... reality is that this is not any
// different than creating a static array (like above)
// but since we will not be reading a file or
// user input for the number we want, we are just
// creating this constant for convenience of
// showing how dynamic allocation works.
const unsigned int MAX_STRIPS = 10;

void setup()
{

  // Every time we want to reference the objects
  // being pointed to by pMyStrips we should
  // test that the value is the state we want.
  // In this case we are at initialization and
  // we only want to create new strips if we
  // haven't already done so (e.g., if there
  // was a possibility that we would initialize
  // more than once)
  if (pMyStrips == NULL)
  {
    // Run-time construction/creation of objects
    // can either single or arrays. The array
    // dimension can be hard coded or can be
    // defined at run-time (e.g., user input)
    // In either case we are now responsible for
    // knowing how many we created because we
    // will not be able to use the sizeof(p)/sizeof(p[0])
    // trick to find how many
    //
    // In this case MAX_STRIPS is a constant that
    // we can use elsewhere in the code to ensure
    // we only reference 0...(MAX_STRIPS - 1)
    // But MAX_STRIPS could just as easily be an
    // input variable defining the size.
    //
    // When we are done with pMyStrips we will
    // be required to delete it... but we will
    // discuss that MUCH later.
    //
    // For now, just remember that these objects
    // will be created in a memory space known
    // as "heap"
    //
    // The following construction only calls the default constructor
    // (if one exists)... this means that while the
    // objects have been constructed they will be initialized
    // with default values... i.e., you will need to
    // call special accessor functions to set up the parameters
    // of each strip... generally this is less convenient
    // structurally, but more convenient if you need to
    // decide things like number of pixels, pin, and type
    // on the fly.
    pMyStrips = new Adafruit_NeoPixel[MAX_STRIPS];

    // If the above pointer comes back NULL, then
    // the allocation failed (i.e., you ran out of
    // heap memory)
  }
  
  strip.begin();
  strip.setBrightness(32);
  strip.show(); // Initialize all pixels to 'off'
}

void loop()
{
  
}
