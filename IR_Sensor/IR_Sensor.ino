/*
  Analog Input
 Demonstrates analog input by reading an analog sensor on analog pin 0 and
 turning on and off a light emitting diode(LED)  connected to digital pin 13.
 The amount of time the LED will be on and off depends on
 the value obtained by analogRead().

 The circuit:
 * Potentiometer attached to analog input 0
 * center pin of the potentiometer to the analog pin
 * one side pin (either one) to ground
 * the other side pin to +5V
 * LED anode (long leg) attached to digital output 13
 * LED cathode (short leg) attached to ground

 * Note: because most Arduinos have a built-in LED attached
 to pin 13 on the board, the LED is optional.


 Created by David Cuartielles
 modified 30 Aug 2011
 By Tom Igoe

 This example code is in the public domain.

 http://www.arduino.cc/en/Tutorial/AnalogInput

 */

int sensorPin = A0;    // select the input pin for the potentiometer
int ledPin = 13;      // select the pin for the LED
int sensorValue = 0;  // variable to store the value coming from the sensor

void setup() 
{
  // declare the ledPin as an OUTPUT:
  pinMode(ledPin, OUTPUT);
  Serial.begin(115200);
  while (!Serial)
  {
    ; // wait for port to open
  }
}

float distance_cm = 0;
const float k = 142.85714285714286;

void loop() 
{
  // read the value from the sensor:
  sensorValue = analogRead(sensorPin);

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
  
  distance_cm = k / ((float)(sensorValue)/204.8 - 1.0);
  
  Serial.print(distance_cm);
  Serial.print(" ");
  
  for (int i = 0; i < (int)(distance_cm / 10); ++i)
  {
    Serial.print("*");
  }
  Serial.println("");
  
  // turn the ledPin on
  //digitalWrite(ledPin, HIGH);
  // stop the program for <sensorValue> milliseconds:
  //delay(sensorValue);
  // turn the ledPin off:
  //digitalWrite(ledPin, LOW);
  // stop the program for for <sensorValue> milliseconds:
  //delay(sensorValue);
}
