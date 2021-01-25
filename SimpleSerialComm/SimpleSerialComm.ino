/*
  Simple Serial Comm example

  Choose LLAP or SensorML as standard?
 */
void setup() 
{
  //Initialize serial and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // prints title with ending line break
  Serial.println(">Arduino");
}

String str;
void loop() 
{
  str = Serial.readStringUntil('\n');

  if (str != "")
  {
    if (str == "<Hello>")
    {
      Serial.println(">Hello Again");
    }
    else if (str.substring(0,3) == "<N>")
    {
      long x = str.substring(3).toInt();
      Serial.print(">N = ");
      Serial.println(x);
    }
    else
    {
      Serial.println(">?");
    }
  }
}
