#include <Wire.h>
#include <Motor.h>
#include <IMU.h>
// #include <SoftPWM.h> 

#define MAXANGLE 70
#define MOTORDEADZONE 70
IMU * imu;
Motor *motorLeft,*motorRight;

float angle, angularVelocity,accelerometerAngle;
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
  Serial.println("Beginning");
  imu = new IMU();
  imu->Calibrate(1000);
  motorSpeed=0;
  motorLeft = new Motor(6,7,8);
  motorRight = new Motor(11,10,9);
  Serial.println("Ending Setup");
}


int current = -255;

void loop()
{

  // Serial.println(current);
  // setMotorSpeed(current,current);
  // delay(10);


  imu->getCurAngleKalman(angle, angularVelocity,accelerometerAngle);
  Serial.print("angle : ");
  Serial.print(angle);
  Serial.print("angularVelocity : ");
  Serial.println(angularVelocity);
  // angle +=balanceAngle;
  // motorSpeed = KP*angle + KD*angularVelocity + KI * motorSpeed;

  // //point of no return
  // if(angle+balanceAngle>MAXANGLE || angle+balanceAngle<-MAXANGLE)
  // {
  //   motorSpeed = 0;
  // } 

  // if(motorSpeed > maxPower)
  // {
  //   motorSpeed = maxPower;
  // }
  // else if(motorSpeed < -maxPower)
  // {
  //   motorSpeed = -maxPower; 
  // }

  // setMotorSpeed((-1)*motorSpeed ,(-1)*motorSpeed);    


  // delay(2);

}


















