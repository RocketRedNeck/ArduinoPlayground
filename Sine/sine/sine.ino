int ledPin = 9;    // LED connected to digital pin 9
int spkrPin = 10;

void setup() {
  // nothing happens in setup
}

long long i = 0;
void loop() 
{
  float x = float(i)/100.0;
  short s = short((sin(x) + 1.0) * 127.0);
  analogWrite(ledPin, s);
  analogWrite(spkrPin, s);
  delay(1);
  ++i;
}
