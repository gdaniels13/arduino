#include <SoftwareSerial.h>

SoftwareSerial* mySerial;
void setup()
{
	mySerial = new SoftwareSerial(2,3);
  Serial.begin(115200);
  
  mySerial->begin(57600);
}

void loop()
{
mySerial->write("hello\n");
delay(1000);
}
