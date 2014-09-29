#ifndef MOTOR_H_
#define MOTOR_H_

class Motor
{
private:
	int in1,in2,pwm;
	int direction;
public:
	Motor();
	Motor(int,int,int); //constructor with pin numbers enable, in1,in2
	void forward(int); //forward values between 0 and 255
	void backward(int); //back  "             "
	void stop();	//stop the motor
	void setspeed(int); 
	void MoveF(float in);
	int getDirection();
	void move(int);
};

#endif

