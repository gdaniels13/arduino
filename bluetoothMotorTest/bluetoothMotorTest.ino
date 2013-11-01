
#include <SoftwareSerial.h>
#include <BalanduinoBluetooth.h>

BalanduinoBluetooth* blue;

void setup() 
{
  Serial.begin(9600);
  blue = new BalanduinoBluetooth();

}

void loop() 
{
 blue->updateValues();
 showValues();
 delay(2);
}


void showValues()
{
	Serial.print("Speed: ");
  	Serial.print(blue->getSpeed());
  	Serial.print(" Direction: ");
  	Serial.print(blue->getDirection()); 
  	Serial.print(" KP: ");
  	Serial.print(blue->getKp());
  	Serial.print(" KI: ");
  	Serial.print(blue->getKi());
  	Serial.print(" KD: ");
  	Serial.println(blue->getKd());
  	// Serial.print(" Target: ");
  	// Serial.println(blue->getTarget());
  	 
}