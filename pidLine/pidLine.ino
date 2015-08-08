#include <QTRSensors.h>
#include <SoftPWM.h>
#include <Motor.h>
#define Kp .8 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 0.01 // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define rightMaxSpeed 80 // max speed of the robot
#define leftMaxSpeed 80  // max speed of the robot
#define rightBaseSpeed 60 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 60  // this is the speed at which the motors should spin when the robot is perfectly on the line
#define NUM_SENSORS  6     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN   255     // emitter is controlled by digital pin 2

#define rightMotor1 5
#define rightMotor2 4
#define rightMotorPWM 3
#define leftMotor1 8
#define leftMotor2 7
#define leftMotorPWM 6
// #define motorPower 8

Motor rightMotor(3,4,5);
Motor leftMotor(6,7,8);

// QTRSensorsRC qtrrc((unsigned char[]) {  14, 15, 16, 17, 18, 19} ,NUM_SENSORS, TIMEOUT, EMITTER_PIN); // sensor connected through analog pins A0 - A5 i.e. digital pins 14-19
QTRSensorsRC qtrrc((unsigned char[]) {  19, 18, 17, 16, 15, 14} ,NUM_SENSORS, TIMEOUT, EMITTER_PIN); // sensor connected through analog pins A0 - A5 i.e. digital pins 14-19

unsigned int sensorValues[NUM_SENSORS];

void setup()
{
 
  int i;
for (int i = 0; i < 100; i++) // calibrate for sometime by sliding the sensors across the line, or you may use auto-calibration instead

  /* comment this part out for automatic calibration 
  if ( i  < 25 || i >= 75 ) // turn to the left and right to expose the sensors to the brightest and darkest readings that may be encountered
     turn_right();  
   else
     turn_left(); */ 
   qtrrc.calibrate();   
   delay(20);
wait();  
delay(2000); // wait for 2s to position the bot before entering the main loop 
    
    Serial.begin(9600);
    /* comment out for serial printing
    
    for (int i = 0; i < NUM_SENSORS; i++)
    {
      Serial.print(qtrrc.calibratedMinimumOn[i]);
      Serial.print(' ');
    }
    Serial.println();

    for (int i = 0; i < NUM_SENSORS; i++)
    {
      Serial.print(qtrrc.calibratedMaximumOn[i]);
      Serial.print(' ');
    }
    Serial.println();
    Serial.println();
    */
  } 

int lastError = 0;

void loop()
{

  unsigned int sensors[6];
  int position = qtrrc.readLine(sensors); // get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
  int error = position - 2500;
  Serial.print(position);
  Serial.print ("\t");
  Serial.print(error);
  Serial.print ("\t");
  //adjust the error if we hit a corner
 //  bool bools[NUM_SENSORS];
	// for(int i=0; i<NUM_SENSORS; i++){
	//     bools[i] = values[i]<THRESH;
	// }

	// if(arrayEquals(bools,(bool[]){0,0,0,0,0}))
	
  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;

  int rightMotorSpeed = rightBaseSpeed - motorSpeed;
  int leftMotorSpeed = leftBaseSpeed + motorSpeed;
  
  if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0) rightMotorSpeed = 0; // keep the motor speed positive
  if (leftMotorSpeed < 0) leftMotorSpeed = 0; // keep the motor speed positive

	if(position==0){
		leftMotorSpeed = leftMaxSpeed;
		rightMotorSpeed = rightBaseSpeed;
	}
	else if (position ==5000){
		leftMotorSpeed = leftBaseSpeed;
		rightMotorSpeed = rightMaxSpeed;	
	}

  	leftMotor.forward(leftMotorSpeed);
  	rightMotor.forward(rightMotorSpeed);

  // // digitalWrite(motorPower, HIGH); // move forward with appropriate speeds
  // digitalWrite(rightMotor1, HIGH);
  // digitalWrite(rightMotor2, LOW);
  // analogWrite(rightMotorPWM, rightMotorSpeed);
  // // digitalWrite(motorPower, HIGH);
  // digitalWrite(leftMotor1, HIGH);
  // digitalWrite(leftMotor2, LOW);
  // analogWrite(leftMotorPWM, leftMotorSpeed);

	Serial.print(leftMotorSpeed);
	Serial.print("\t:\t");
	Serial.print(rightMotorSpeed);
	Serial.println();
}	
  
void wait(){
    //digitalWrite(motorPower, LOW);
}

bool arrayEquals(bool* array1,bool*array2){
	for(int i=0; i<NUM_SENSORS; i++){
	    if(array1[i]!=array2[i]){
	    	return false;
	    }
	}
	return true;
}