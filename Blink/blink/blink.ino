int ledPin = 9;    // LED connected to digital pin 9
int spkrPin = 10;

void setup() {
  // nothing happens in setup
}

short n = 0;
short s = 0;

void loop() 
{
  analogWrite(ledPin, s*255);
  analogWrite(spkrPin, s*255);
  delay(n);
  s = (s == 0?1:0);
  n = (n+1) % 50;
  n = 5;
}
