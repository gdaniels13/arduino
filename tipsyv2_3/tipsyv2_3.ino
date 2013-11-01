#include <Event.h>
#include <Timer.h>

#include <PID_v1.h>
#include <IMU.h>
#include <Kalman.h>
#include <Motor.h>
#include <Wire.h>



Motor *motorLeft,*motorRight;
IMU * imu;
Timer timer;


double angle=0;
double motorSpeed=0;
double setPoint=6; 

double kp=15;
double ki=7;
double kd=3;
float angularVelocity;

PID myPID(&angle,&motorSpeed,&setPoint,kp,ki,kd,DIRECT);
int thingID;

void setup()
{
  Serial.begin(9600);
  //initialize the imu
  imu = new IMU();

  thingID = timer.every(2,theISR);
  
  //initialize motor
  motorLeft = new Motor(9,4,5);
  motorRight = new Motor(10,6,7);
  
  //initialize PID
  myPID.SetSampleTime(2);
  myPID.SetMode(AUTOMATIC);
}


void loop()
{
  timer.update();
  imu->getCurAngleKalman(angle,angularVelocity);
  setMotorSpeed(motorSpeed,motorSpeed, myPID.GetDirection());
//    Serial.print(angle);
//  Serial.print("  ");
//  Serial.println(motorSpeed);
}

void theISR()
{
  imu->getCurAngleKalman(angle, angularVelocity);
  myPID.Compute();

 // digitalWrite( 8, digitalRead( 8 ) ^ 1 );
       
}

void setMotorSpeed(int left, int right, int dirrection)
{
  if(dirrection==0) 
  {
   left*=-1;
    right*=-1; 
  }
    
  motorLeft->setspeed(left);
  motorRight->setspeed(right);
}
