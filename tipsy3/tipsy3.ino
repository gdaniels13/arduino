
#include "BalanduinoBluetooth.h"
#include <cstdlib.h>
#include <Wire.h>
#include <IMU.h>
#include <Kalman.h>

BalanduinoBluetooth* blueTooth;



IMU* imu;

float angle;
float accelAngle;
float gyro;
float targetAngle;

void setup()
{
	blueTooth = new BalanduinoBluetooth();
}

void loop() 
{

}

