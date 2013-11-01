#include <SoftwareSerial.h>

SoftwareSerial* mySerial;
void setup()
{
	mySerial = new SoftwareSerial(12,13);
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
