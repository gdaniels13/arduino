
#include <Motor.h>
#include "pwmsetter.h"


Motor m(10,9,8);


void setup() 
{
	setPwmFrequency( 10, 2);
	Serial.begin(9600);
	m.forward(255);
}

void loop() 
{
	m.forward(255);
}