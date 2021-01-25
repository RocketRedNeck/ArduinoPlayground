// ****************************************************************************************************
//
// IR Sensor Enable/Disable
//
// The following example demonstrates using output pins to enable/disable a specific IR Sensor
//
// This is useful when needing to switch between range gate/resolution via two different
// sensors that would intefere with each other if both were enabled.
//
// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------
// Copyright (c) 2016 - RocketRedNeck.net
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
// and rather than tempt fate we just revert to old habit.
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
  
  DP_IR_100_500    = 4,    // Assigned as our IR Sensor Enable (Power)
  
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
  AP_IR_100_500   = 0,
  AP_UNUSED1      = 1,
  AP_UNUSED2      = 2,
  AP_UNUSED3      = 3,
  AP_UNUSED4      = 4,
  AP_UNUSED5      = 5,
  
  AP_MAX_NUMBER_OF_PINS,
  AP_BUGUS
  
};

enum OnOffValues
{
  OFF = 0,
  ON  = 1
};

int sensorValue = 0;  // variable to store the value coming from the sensor

void setup() 
{
  // declare the ledPin as an OUTPUT:
  pinMode(DP_BUILT_IN_LED, OUTPUT);
  pinMode(DP_IR_100_500,   OUTPUT);
  Serial.begin(115200);
  while (!Serial)
  {
    ; // wait for port to open
  }
}

float distance_cm = 0;


// IR Sensor GP2Y0A710K (100 - 550 cm) has relationships of 
// 2.5V approximately 100 cm
// 1.5V at approximately 333 cm
//
// The above is approximatley LINEAR as V = k/L + 1 such that L = k/(V - 1)
//
// k = (2.5 - 1.5)/(0.01 - 0.003) = 142.85714285714286
//
// Sensor voltage is determined analog value (As):  Vs = 5.0 * (As/1024)
//
// L = k / (5.0 * (As/1024) - 1) --> k / ((As / 204.8) - 1) --> k ((10 * As / 2048) - 1)
const float K  =    142.85714285714286;
#define S_TO_V 0.0048828125        // (1.0/204.8)

int count = 0;
bool sensorOn = false;

void loop() 
{
  // Every Nth pass, toggle
  if (0 == (++count % 1000))
  {
    sensorOn = ! sensorOn;
    
    if ( sensorOn)
    {
      digitalWrite(DP_BUILT_IN_LED, ON);
      digitalWrite(DP_IR_100_500, ON);

      // When turning on Sensor, wait for stable result
      delay(26);  // miliseconds per spec for this sensor
      
    }
    else
    {
      digitalWrite(DP_BUILT_IN_LED, OFF);
      digitalWrite(DP_IR_100_500, OFF);
    }
  }

  if (sensorOn)
  {
    // read the value from the sensor:
    sensorValue = analogRead(AP_IR_100_500);
  
  
    float volts = (float)sensorValue / 204.8;
  
    bool tooClose = (volts > 2.5);
    bool tooFar   = (volts < 1.4);
  
    if (! tooClose && ! tooFar)
    {
    
      distance_cm = K / (volts - 1.0);
      
      Serial.print(distance_cm);
      Serial.print(" ");
      
      for (int i = 0; i < (int)(distance_cm / 10); ++i)
      {
        Serial.print("*");
      }
    }
    else if (tooClose)
    {
      Serial.print("Too CLOSE");
    }
    else
    {
      Serial.print("Too FAR");
    }
  }
  else
  {
    Serial.print("Sensor OFF");
  }
  Serial.println("");
  
}
