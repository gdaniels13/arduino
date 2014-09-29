#include "Motor.h"
#include <Arduino.h> 

Motor::Motor()
{

}

Motor::Motor(int a,int b,int c) //constructor with pin numbers enable, in1,in2
{
	pwm = a;
	in1 = b;
	in2 = c;
	pinMode(in1,OUTPUT);
	pinMode(in2,OUTPUT);
	stop();
}


 void Motor::forward(int in) //forward values between 0 and 255
{
	digitalWrite(in1, LOW);
	digitalWrite(in2, HIGH);
	
	if(in>255) in = 255;
	if(in<0) in = 0;
	analogWrite(pwm,in);
	direction = 1;
}

void Motor::MoveF(float in) //float between 1 and -1 1 for forward -1 for back
{
	if(in<0)
	{
		backward((int)((-1)*in*255));
	}
	else
	{
		forward((int) (in*255));
	}
}

void Motor::move(int in){
	if(in<0)
	{
		backward((int)((-1)*in)-120);
	}
	else
	{
		forward((int) (in + 120));
	}	
}



void Motor::backward(int in) //back  "             "
{
	digitalWrite(in2, LOW);
	digitalWrite(in1, HIGH);
	
	if(in>255) in = 255;
	if(in<0) in = 0;
	analogWrite(pwm,in);
	direction =0;
}

void Motor::setspeed(int in)
{
	if(in >0)
	forward(in);
	else
	backward((-1)*in);

}

void Motor::stop()	//stop the motor
{
	analogWrite(pwm,0);
	digitalWrite(in2, LOW);
	digitalWrite(in1, LOW);
}

int Motor::getDirection(){
	return direction;
}
