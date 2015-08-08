
#include "AccelStepper.h"
// accelStepperTest.ino
#define LEFT_MOTOR_DIR 7
#define LEFT_MOTOR_STEP 6
//pin definitions 
#define MOTOR_PORT PORTD
#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1
#define LEFT_MOTOR_DIR 7
#define LEFT_MOTOR_STEP 6
#define RIGHT_MOTOR_DIR 5
#define RIGHT_MOTOR_STEP 4
#define MOTOR_ENABLE 2
#define MS1 3
#define MS2 8
#define MS3 12


AccelStepper lmotor(AccelStepper::DRIVER,LEFT_MOTOR_STEP,LEFT_MOTOR_DIR);
AccelStepper rmotor(AccelStepper::DRIVER,RIGHT_MOTOR_STEP,RIGHT_MOTOR_DIR);

void motorInit(){
	pinMode(MS1, OUTPUT);	
	pinMode(MS2, OUTPUT);	
	pinMode(MS3, OUTPUT);	
	// pinMode(LEFT_MOTOR_STEP, OUTPUT);	
	// pinMode(LEFT_MOTOR_DIR, OUTPUT);	
	// pinMode(MOTOR_ENABLE, OUTPUT);	
	// pinMode(RIGHT_MOTOR_STEP, OUTPUT);	
	// pinMode(RIGHT_MOTOR_DIR, OUTPUT);	
	digitalWrite(MS1, HIGH);
	digitalWrite(MS2, HIGH);
	digitalWrite(MS3, LOW);
	digitalWrite(MOTOR_ENABLE,LOW);
}

void setup() {
	motorInit();
	lmotor.setMaxSpeed(2000);
	lmotor.setSpeed(500);
	lmotor.setAcceleration(100);
	rmotor.setMaxSpeed(2000);
	rmotor.setSpeed(500);
	rmotor.setAcceleration(100);

}

void loop() {
    lmotor.runSpeed();
    rmotor.runSpeed();
}

