// DualMotorShield.pde
// -*- mode: C++ -*-
//
// Shows how to run 2 simultaneous steppers
// using the Itead Studio Arduino Dual Stepper Motor Driver Shield
// model IM120417015
// This shield is capable of driving 2 steppers at 
// currents of up to 750mA
// and voltages up to 30V
// Runs both steppers forwards and backwards, accelerating and decelerating
// at the limits.
//
// Copyright (C) 2014 Mike McCauley
// $Id:  $

#include <AccelStepper.h>
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
// The X Stepper pins
#define STEPPER1_DIR_PIN 7
#define STEPPER1_STEP_PIN 6
// The Y stepper pins
#define STEPPER2_DIR_PIN 5
#define STEPPER2_STEP_PIN 4

// Define some steppers and the pins the will use
AccelStepper stepper1(AccelStepper::DRIVER, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);
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
void setup()
{  
    motorInit();
    stepper1.setMaxSpeed(3000.0);
    stepper1.setAcceleration(500.0);
    stepper1.moveTo(100000);
    
    stepper2.setMaxSpeed(3000.0);
    stepper2.setAcceleration(500.0);
    stepper2.moveTo(100000);
}

void loop()
{
    // Change direction at the limits
    if (stepper1.distanceToGo() == 0)
	stepper1.moveTo(-stepper1.currentPosition());
    if (stepper2.distanceToGo() == 0)
	stepper2.moveTo(-stepper2.currentPosition());
    stepper1.run();
    stepper2.run();
}
