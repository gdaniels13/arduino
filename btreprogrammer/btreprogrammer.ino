#include <SoftwareSerial.h>

SoftwareSerial* mySerial;
void setup()
{
	mySerial = new SoftwareSerial(12,13);
  Serial.begin(115200);
  
  mySerial->begin(9600);
}

void loop()
{


  if (mySerial->available())
     Serial.write(mySerial->read());
   if (Serial.available())
     mySerial->write(Serial.read());
}
