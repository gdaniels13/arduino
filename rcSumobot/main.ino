
#include <Motor.h>
#include "pwmsetter.h"
#define left 1
#define right 0
#define center 260
#define slop 10

Motor leftMotor(10,11,12);
Motor rightMotor(9,8,7);
int RXCH[2]; 
volatile int RXSG[2];
int RXOK[2];
int PWMSG[2];

void setup() 
{
	Serial.begin(115200);
  RXCH[left] = 4;  //Throttle
  RXCH[right] = 3;
  for (int i = 0; i < 2; i++){
    pinMode(RXCH[i], INPUT);
  }
}

int readFromReceiver(int i){
 	RXSG[i] = pulseIn(RXCH[i], HIGH, 20000);                            //read the receiver signal
  if (RXSG[i] == 0) 
  {
  	RXSG[i] = RXOK[i];
  } 
  else 
  {
  	RXOK[i] = RXSG[i];
  }    //if the signal is good then use it, else use the previous signal
  
  PWMSG[i] = map(RXSG[i], 1000, 2000, 0, 511);                        //substitute the high values to a value between 0 and 511
  constrain (PWMSG[i], 0, 511);
  return PWMSG[i];
}

int throttle;
int dirrection;
int leftIn;
int rightIn;
int leftMotorScaled,rightMotorScaled,maxMotorScale,leftMotorScale,rightMotorScale;
void loop()
{
	leftIn = readFromReceiver(left);
	rightIn = readFromReceiver(right);
	Serial.print(leftIn);
	Serial.print("|");
	Serial.println(rightIn);
	if(leftIn>slop || leftIn < -slop){
		if(leftIn>0){
			leftMotor.forward(leftIn);
		}
		else{
			leftMotor.backward(abs(leftIn));
		} 
	}
	else{
		leftMotor.stop();
	}

	if(rightIn>slop || rightIn < -slop){
		if(rightIn>0){
			rightMotor.forward(rightIn);
		}
		else{
			rightMotor.backward(abs(rightIn));
		} 
	}
	else{
		rightMotor.stop();
	}

	delay(10);
}




