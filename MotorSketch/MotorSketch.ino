
#include <Motor.h>


Motor m(9,8,7);

int led = 13;


void setup() 
{
	Serial.begin(9600);
	m.forward(100);
	delay(1000);
	m.forward(1000);
  pinMode(led, OUTPUT);     
}

void loop() 
{
	  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);               // wait for a second
  digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);
}