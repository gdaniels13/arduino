#include <Wire.h>
#include <Motor.h>
#include <IMU.h>

#define MAXANGLE 70
#define MOTORDEADZONE 70
IMU * imu;
Motor *motorLeft,*motorRight;

float angle, angularVelocity;
float motorSpeed;
float KP = 15;
float KI = 6;
float KD = 9;
float balanceAngle=6;

float maxPower = 200 ;



void setMotorSpeed(int left, int right)
{
  motorLeft->setspeed(left);
  motorRight->setspeed(right);
}



void setup()
{
  Serial.begin(9600);
  imu = new IMU();
  motorSpeed=0;
  motorLeft = new Motor(6,7,8);
  motorRight = new Motor(11,10,9);

}




void loop()
{
  imu->getCurAngleKalman(angle, angularVelocity);
  angle +=balanceAngle;
  motorSpeed = KP*angle + KD*angularVelocity + KI * motorSpeed;

  //point of no return
  if(angle+balanceAngle>MAXANGLE || angle+balanceAngle<-MAXANGLE)
  {
    motorSpeed = 0;
  } 

  if(motorSpeed > maxPower)
  {
    motorSpeed = maxPower;
  }
  else if(motorSpeed < -maxPower)
  {
    motorSpeed = -maxPower; 
  }

  setMotorSpeed((-1)*motorSpeed ,(-1)*motorSpeed);    


  delay(2);

}


















