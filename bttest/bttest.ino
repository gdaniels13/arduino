
#include <Wire.h>
#include <IMU.h>
#include <Kalman.h>
#include <BalanduinoBluetooth.h>
 #include <Motor.h>
 #include <SoftwareSerial.h>
 #include <string>
 #include <cstdlib.h>


#define MOTORRANGE 255


// IMU* imu;
BalanduinoBluetooth * blue;
 // Motor MotorLeft(5,6,7);
 // Motor MotorRight(10,9,8);

 float angle;
 float accelAngle;
 float gyro;
 float motorSpeed = 0;
 unsigned long lastTransmission = 0;

void setup() 
{
	blue = new BalanduinoBluetooth();
	// imu = new IMU();
}

void loop() 
{
	//imu->getCurAngleKalman(angle,gyro,accelAngle);
	blue->updateValues();
	
	showValues();
	// simpleRC();
  //	updateGraph();

 	//PID();
 	delay(20);
}

// void PID()
// {
// 	motorSpeed = blue->getKp()*angle + blue->getKd()*gyro + blue->getKi()*motorSpeed;

// 	if (motorSpeed>MOTORRANGE)
// 	{
// 		motorSpeed=MOTORRANGE; 
// 	}
// 	else if(motorSpeed< -MOTORRANGE)
// 	{
// 		motorSpeed=-MOTORRANGE;
// 	}

// 	MotorLeft.setspeed((int)motorSpeed);
// 	MotorRight.setspeed((int)motorSpeed);
// }

void updateGraph()
 {
 	if(millis() - lastTransmission > 10 && blue->getGraphBool())
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
  	// Serial.print(" Target: ");
  	// Serial.println(blue->getTarget());
  	 
}

// void simpleRC()
//  {
//  	float direction = blue->getDirection();
//  	float speed = blue->getSpeed();
//  	float left, right;
//  	if(!(direction < .05 && direction > -.05))
//  	{
//  		left = speed - (1-speed)*direction;
//  		right =speed + (1-speed)*direction;
//  	}

// 	MotorLeft.MoveF(left);
//  	MotorRight.MoveF(right);
//  }



