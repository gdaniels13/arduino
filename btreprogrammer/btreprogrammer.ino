#include <SoftwareSerial.h>
#include "config/AltSoftSerial_Boards.h"
#include "config/AltSoftSerial_Timers.h"
#include "config/known_boards.h"
#include "config/known_timers.h"
#include "AltSoftSerial.h"


SoftwareSerial* mySerial;
void setup()
{
	mySerial = new SoftwareSerial(A1,A0);
  Serial.begin(9600);
  
  mySerial->begin(57600);
}

void loop()
{


  if (mySerial->available())
     Serial.write(mySerial->read());
   if (Serial.available())
     mySerial->write(Serial.read());
}
