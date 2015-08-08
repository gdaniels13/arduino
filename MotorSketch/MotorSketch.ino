
#include <Motor.h>
#include <Arduino.h> 


Motor m(11,8,7);

int led = 13;


void setup() 
{
	Serial.begin(9600);
	m.forward(100);
	delay(1000);
	m.forward(255);
	pinMode(led, OUTPUT);  
	analogWrite(9, 255);   
}

void loop() 
{
	digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);               // wait for a second
  digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);

}