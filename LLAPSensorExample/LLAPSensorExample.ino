
#include "LLAPSerial.h"	// include the library

char deviceId[] = "--";

void setup() 
{
  // initialise serial:
  Serial.begin(115200);
  
  // Initialise the LLAPSerial library and set our default ID to -- (unknown)
  LLAP.init(deviceId);

  LLAP.sendMessage(String("STARTED"));

}

String msg;
String reply;

bool cycle = false;
unsigned long count = 0;
unsigned long cycleTime_ms = 1000;
unsigned long schedTime_ms = cycleTime_ms;

void processMessage(void)
{
  // print the string when a newline arrives:
  if (LLAP.bMsgReceived) 
  {
    msg = LLAP.sMessage;
    reply = msg;
    LLAP.bMsgReceived = false;  // if we do not clear the message flag then message processing will be blocked

    // HELLO and CHDEVID are handled by the LLAP message processing automatically
    // and do NOT respond with the device ID
    if (msg.compareTo("ACK------") == 0)
    {
      LLAP.sendMessage(reply);
    }
    else if (msg.compareTo("DEVNAME--") == 0)
    {
      reply = "ARDUINO--";
      LLAP.sendMessage(reply);
    }
    else if (msg.compareTo("CYCLE----") == 0)
    {
      cycle = true;
    }
    else if (msg.compareTo("STOP-----") == 0)
    {
      cycle = false;
    }
    else if (msg.startsWith("INTVL"))
    {
      long intvl = msg.substring(5).toInt();
      if (intvl > 0)
      {
        count = 0;
        switch (msg.substring(8).c_str()[0])
        {
          case 'T':
            cycleTime_ms = intvl;
            break;
          case 'S':
            cycleTime_ms = (intvl * 1000);
            break;
          case 'M':
            cycleTime_ms = (intvl * 60000);
            break;
          case 'H':
            cycleTime_ms = (intvl * 36000000);
          case 'D':
            cycleTime_ms = (intvl * 86400000);
            break;
          default:
            break;
        } // end switch on interval scale
      } // end if interval is positive
    } // end if recognized command
    else
    {
      // Define a generic error message for unknown
      // commands; the offending message is sent back
      reply = "ERROR----";
      LLAP.sendMessage(reply);
      LLAP.sendMessage(msg);
    }
  }   // end if message received
} // end processMessage

void processSchedule(void)
{
  unsigned long aboutNow_ms = millis();
  
  // If cycle mode is enabled then 
  if (cycle)
  {
    if (aboutNow_ms >= schedTime_ms)
    {
      reply = "TICK";
      reply += count++;
      LLAP.sendMessage(reply);

      // Schedule next
      schedTime_ms += cycleTime_ms;
    }
     
  }  
}

void loop() 
{
  processMessage();

  processSchedule();
}




