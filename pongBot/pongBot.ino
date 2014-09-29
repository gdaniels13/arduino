
#include <Motor.h>
#include "pwmsetter.h"

Motor leftMotor(10,11,12);
Motor rightMotor(9,8,7);
char buffer[4];
int ledState=LOW;

class State{
public:
    byte motorADirection;
    byte motorASpeed;
    byte motorBSpeed;
    byte motorBDirection;

   static State fromByteArray(char* bytes);
};

 State State::fromByteArray(char* bytes){
    	State toReturn;
    	toReturn.motorADirection = bytes[0];
    	toReturn.motorASpeed = bytes[1];
    	toReturn.motorBDirection=bytes[2];
    	toReturn.motorBSpeed = bytes[3];
    	return toReturn;
    };


void setup() 
{
	Serial.begin(115200);

}


void process(State state){
	if(state.motorADirection==1){
		leftMotor.forward(state.motorASpeed);
	}
	else if(state.motorADirection==0){
		leftMotor.backward(state.motorASpeed);
	}
	else {
		leftMotor.stop();
	}

	if(state.motorBDirection==1){
		rightMotor.forward(state.motorBSpeed);
	}
	else if(state.motorBDirection==0){
		rightMotor.backward(state.motorBSpeed);
	}
	else{
		rightMotor.stop();
	}
}

void toggleLED(){
	if(ledState == LOW)
	{
		digitalWrite(13, HIGH);
		ledState = HIGH;
	}
	else{
		digitalWrite(13, LOW);
		ledState = LOW;
	}
}

void loop()
{
	if(Serial.available()>=4){
		Serial.readBytes(buffer,4);
		process( State::fromByteArray(buffer));
	}

}