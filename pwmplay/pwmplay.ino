#include <TimerHelpers.h>
#include "pwmsetter.h"


// Timer 0

// output    OC0B   pin 11  (D5)


  
void setup() {
	pinMode(9,OUTPUT);
	pinMode(13, OUTPUT); 
	setPwmFrequency( 9, 1);
	analogWrite(9, 128); 
}  // end of setup

void loop() 
{

}