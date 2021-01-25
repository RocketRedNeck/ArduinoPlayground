const int pingPin = 11;
const int ledPin = 12;
unsigned int duration, inches;

void setup() 
{
  Serial.begin(9600);
}

int averageDistance_inches(int numSamples)
{
  pinMode(pingPin, OUTPUT);          // Set pin to OUTPUT
  digitalWrite(pingPin, LOW);        // Ensure pin is low
  delayMicroseconds(2);

  unsigned int duration = 0;
  unsigned int inches = 0;  
  for (int i = 0; i < numSamples; ++i)
  {
    pinMode(pingPin, OUTPUT);          // Set pin to OUTPUT    
    digitalWrite(pingPin, HIGH);       // Start ranging
    delayMicroseconds(10);              //   with 10 microsecond burst
    digitalWrite(pingPin, LOW);        // End ranging
    
    pinMode(pingPin, INPUT);           // Set pin to INPUT
    duration = pulseIn(pingPin, HIGH); // Read echo pulse
    inches += duration / 74 / 2;        // Convert to inches
    
    // Set this loop up to sample as quickly as possible
    // Speed of sound is slightly under 1000 ft/s
    // Sensor range is between 0 and 10 feet which means
    // that we need to sample no faster than every 10 ms
    // to avoid overlapping pulses. We will sample every
    // 20 ms to be certain that we have waited long enough
    delay(20);		               // Short delay
  }
  
  return inches/numSamples;
}

enum States 
{
  WAITING_FOR_CUSTOMER,
  CUSTOMER_DETECTED,
  WAITING_FOR_CUSTOMER_TO_LEAVE,
  EMPTY_STALL_DETECTED,
  FLUSHING
};

States currentState = WAITING_FOR_CUSTOMER;
long stateTime_ms = 0;
const long MIN_PRESENT_TIME_MS   = 3000;
const long MIN_DEPARTURE_TIME_MS = 5000;
const long MIN_FLUSH_TIME_MS     = 5000;
const int MAX_PRESENT_DISTANCE_INCHES = 24;
const int MIN_DEPARTURE_DISTANCE_INCHES = 60;

void loop() 
{
  int currentDistance_inches = averageDistance_inches(10);
  switch (currentState)
  {
    case WAITING_FOR_CUSTOMER:
    {
      if (currentDistance_inches < MAX_PRESENT_DISTANCE_INCHES)
      {
         stateTime_ms = millis();
         currentState = CUSTOMER_DETECTED;
         Serial.print("Customer Detected...\n");
      }
      break;
    }
    case CUSTOMER_DETECTED:
    {
      if (currentDistance_inches < MAX_PRESENT_DISTANCE_INCHES)
      {
         if (millis() - stateTime_ms > MIN_PRESENT_TIME_MS)
         {
           // Customer has been present for minimum required time
           stateTime_ms = millis();
           currentState = WAITING_FOR_CUSTOMER_TO_LEAVE;
           Serial.print("Waiting for Customer to Leave...\n");
         }
      }
      else
      {
         stateTime_ms = millis();
         currentState = WAITING_FOR_CUSTOMER;
         Serial.print("False Alarm... No Customer\n");
       }
      break;
    }
    case WAITING_FOR_CUSTOMER_TO_LEAVE:
    {
      if (currentDistance_inches > MIN_DEPARTURE_DISTANCE_INCHES)
      {
        stateTime_ms = millis();
        currentState = EMPTY_STALL_DETECTED;
        Serial.print("Empty Stall...\n");
      }
      break;
    }
    case EMPTY_STALL_DETECTED:
    {
      if (currentDistance_inches > MIN_DEPARTURE_DISTANCE_INCHES)
      {
         if (millis() - stateTime_ms > MIN_DEPARTURE_TIME_MS)
         {
           // Stall has been empty for required time
           stateTime_ms = millis();
           currentState = FLUSHING;
           Serial.print("Flushing...\n");
         }
      }
      else
      {
         stateTime_ms = millis();
         currentState = WAITING_FOR_CUSTOMER_TO_LEAVE;
         Serial.print("False Alarm...Customer Still Here\n");
      }      
      break;
    }
    case FLUSHING:
    {
       if (millis() - stateTime_ms > MIN_FLUSH_TIME_MS)
       {
         // Flush has completed
         stateTime_ms = millis();
         currentState = WAITING_FOR_CUSTOMER;
         Serial.print("Flushing Complete\n\n");
       }
       break;
    }
 
  } // end switch on current state
}

