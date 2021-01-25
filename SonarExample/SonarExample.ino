#include <Wire.h>
#include <Servo.h> 
Servo myservo;  
int incomingByte;
int pos = 0;
int reading = 0;
 
void setup() 
{ 
  myservo.attach(9); 
  Serial.begin(9600);
  Wire.begin();
} 
 
 
void loop() 
{ 
  if (Serial.available() > 0) 
  {
    incomingByte = Serial.read();
    if (incomingByte == 'X') 
    {
  for(pos = 0; pos < 180; pos += 5)  // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
  myservo.write(pos);  
  
  Wire.beginTransmission(112); // transmit to device #112 (0x70)
  Wire.send(0x00);             // sets register pointer to the command register (0x00)  
  Wire.send(0x51);             // command sensor to measure in "inches" (0x50) 
                            // use 0x52 for ping microseconds
  Wire.endTransmission();      // stop transmitting
  delay(70);                   // datasheet suggests at least 65 milliseconds
  Wire.beginTransmission(112); // transmit to device #112
  Wire.send(0x02);             // sets register pointer to echo #1 register (0x02)
  Wire.endTransmission();      // stop transmitting
  Wire.requestFrom(112, 2);    // request 2 bytes from slave device #11

  if(2 <= Wire.available())    // if two bytes were received
  {
    reading = Wire.receive();  // receive high byte (overwrites previous reading)
    reading = reading << 8;    // shift high byte to be high 8 bits
    reading |= Wire.receive(); // receive low byte as lower 8 bits
    Serial.println(reading);   // print the reading
  }
    delay(100);                       // waits 15ms for the servo to reach the position 
  } 
    myservo.write(0);
    delay(1000);
    } 
  }
}
