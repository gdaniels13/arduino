
#include <Wire.h>
#include "Motor.h"
#include <SoftwareSerial.h>
#include <string>
#include "BalanduinoBluetooth.h"

#include <cstdlib.h>
#include "IMU.h"
#include "Kalman.h"



IMU* imu;
BalanduinoBluetooth * blue;
Motor MotorLeft(10,7,6);
Motor MotorRight(9,4,5);

float angle;
float accelAngle;
float gyro;
unsigned long lastTransmission = 0;

void setup() 
{
	Serial.begin(9600);
	blue = new BalanduinoBluetooth();
	imu = new IMU();
}

void loop() 
{
	// Serial.println("hello");
	// MotorLeft.forward(255);
	// MotorRight.forward(255);
	// delay(500);
	// MotorRight.backward(255);
	// MotorLeft.backward(255);
	// delay(500);

	imu->getCurAngleKalman(angle,gyro,accelAngle);
	blue->updateValues();
 	// showValues();
   	simpleRC();
  	//updateGraph();
}

void updateGraph()
{
	if(millis() - lastTransmission > 50 && blue->getGraphBool())
	{
		lastTransmission = millis();
		blue->printGraph(accelAngle,gyro,angle);
	}
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
//   	Serial.print(" Target: ");
//   	Serial.println(blue->getTarget());
}








void simpleRC()
{
	float direction = blue->getDirection();
	float speed = blue->getSpeed();
	float left, right;
	if(!(direction < .05 && direction > -.05))
	{
		left = speed - (1-speed)*direction;
		right =speed + (1-speed)*direction;
	}

	MotorLeft.MoveF(speed);
	MotorRight.MoveF(speed);
}



