#include "IMU.h"
#include <Wire.h>
#include "math.h"
#include <Arduino.h>

IMU::IMU()
{  
    //Serial.println("imu init");
	Wire.begin();
	angle = 0;
	MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0);
    updateValues();
	lastUpdate = micros();
}


void IMU::Calibrate()
{
	gyroOffset = 0;
	for( int i =0; i<1000; ++i)
	{
	  accel_t_gyro_union accel_t_gyro;

	  
	  MPU6050_read (MPU6050_ACCEL_XOUT_H, (uint8_t *) &accel_t_gyro, sizeof(accel_t_gyro));
	  
	  uint8_t swap;
	  


	  SWAP (accel_t_gyro.reg.x_accel_h, accel_t_gyro.reg.x_accel_l);
	  SWAP (accel_t_gyro.reg.y_accel_h, accel_t_gyro.reg.y_accel_l);
	  SWAP (accel_t_gyro.reg.z_accel_h, accel_t_gyro.reg.z_accel_l);
	  SWAP (accel_t_gyro.reg.t_h, accel_t_gyro.reg.t_l);
	  SWAP (accel_t_gyro.reg.x_gyro_h, accel_t_gyro.reg.x_gyro_l);
	  SWAP (accel_t_gyro.reg.y_gyro_h, accel_t_gyro.reg.y_gyro_l);
	  SWAP (accel_t_gyro.reg.z_gyro_h, accel_t_gyro.reg.z_gyro_l);
	  gyroOffset = accel_t_gyro.value.x_gyro;
	  accelxoffset = accel_t_gyro.value.x_accel;
	  accelyoffset = accel_t_gyro.value.y_accel;
	  accelzoffset = accel_t_gyro.value.z_accel;
	}

	gyroOffset /=1000;
	accelxoffset /=1000;
	accelyoffset /=1000;
	accelzoffset /=1000;
}


void IMU::GetRawAccel(int& x, int& y, int & z)
{
  
	accel_t_gyro_union  accel_t_gyro;
	  
	  MPU6050_read (MPU6050_ACCEL_XOUT_H, (uint8_t *) &accel_t_gyro, sizeof(accel_t_gyro));
	  
	  uint8_t swap;
	  
	  SWAP (accel_t_gyro.reg.x_accel_h, accel_t_gyro.reg.x_accel_l);
	  SWAP (accel_t_gyro.reg.y_accel_h, accel_t_gyro.reg.y_accel_l);
	  SWAP (accel_t_gyro.reg.z_accel_h, accel_t_gyro.reg.z_accel_l);
	  SWAP (accel_t_gyro.reg.t_h, accel_t_gyro.reg.t_l);
	  SWAP (accel_t_gyro.reg.x_gyro_h, accel_t_gyro.reg.x_gyro_l);
	  SWAP (accel_t_gyro.reg.y_gyro_h, accel_t_gyro.reg.y_gyro_l);
	  SWAP (accel_t_gyro.reg.z_gyro_h, accel_t_gyro.reg.z_gyro_l);
	  
	  x= accel_t_gyro.value.x_accel;
	  z= accel_t_gyro.value.z_accel;
	  y= accel_t_gyro.value.y_accel;
}

void IMU::GetRawGyro(int& x, int& y, int & z)
{
	accel_t_gyro_union  accel_t_gyro;
	  
	  MPU6050_read (MPU6050_ACCEL_XOUT_H, (uint8_t *) &accel_t_gyro, sizeof(accel_t_gyro));
	  
	  uint8_t swap;
	  


	  SWAP (accel_t_gyro.reg.x_accel_h, accel_t_gyro.reg.x_accel_l);
	  SWAP (accel_t_gyro.reg.y_accel_h, accel_t_gyro.reg.y_accel_l);
	  SWAP (accel_t_gyro.reg.z_accel_h, accel_t_gyro.reg.z_accel_l);
	  SWAP (accel_t_gyro.reg.t_h, accel_t_gyro.reg.t_l);
	  SWAP (accel_t_gyro.reg.x_gyro_h, accel_t_gyro.reg.x_gyro_l);
	  SWAP (accel_t_gyro.reg.y_gyro_h, accel_t_gyro.reg.y_gyro_l);
	  SWAP (accel_t_gyro.reg.z_gyro_h, accel_t_gyro.reg.z_gyro_l);
	  
	  x= accel_t_gyro.value.x_gyro;
	  z= accel_t_gyro.value.z_gyro;
	  y= accel_t_gyro.value.y_gyro;
}

void IMU::updateValues()
{
	  MPU6050_read (MPU6050_ACCEL_XOUT_H, (uint8_t *) &lastSensorValues, sizeof(lastSensorValues));
	  
	  uint8_t swap;
	  
	  SWAP (lastSensorValues.reg.x_accel_h, lastSensorValues.reg.x_accel_l);
	  SWAP (lastSensorValues.reg.y_accel_h, lastSensorValues.reg.y_accel_l);
	  SWAP (lastSensorValues.reg.z_accel_h, lastSensorValues.reg.z_accel_l);
	  SWAP (lastSensorValues.reg.t_h, lastSensorValues.reg.t_l);
	  SWAP (lastSensorValues.reg.x_gyro_h, lastSensorValues.reg.x_gyro_l);
	  SWAP (lastSensorValues.reg.y_gyro_h, lastSensorValues.reg.y_gyro_l);
	  SWAP (lastSensorValues.reg.z_gyro_h, lastSensorValues.reg.z_gyro_l);
}


// --------------------------------------------------------
// MPU6050_write
//
// This is a common function to write multiple bytes to an I2C device.
//
// If only a single register is written,
// use the function MPU_6050_write_reg().
//
// Parameters:
//   start : Start address, use a define for the register
//   pData : A pointer to the data to write.
//   size  : The number of bytes to write.
//
// If only a single register is written, a pointer
// to the data has to be used, and the size is
// a single byte:
//   int data = 0;        // the data to write
//   MPU6050_write (MPU6050_PWR_MGMT_1, &c, 1);
//
int IMU::MPU6050_write(int start, const uint8_t *pData, int size)
{
  int n, error;

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);        // write the start address
  if (n != 1)
    return (-20);

  n = Wire.write(pData, size);  // write data bytes
  if (n != size)
    return (-21);

  error = Wire.endTransmission(true); // release the I2C-bus
  if (error != 0)
    return (error);

  return (0);         // return : no error
}

// --------------------------------------------------------
// MPU6050_write_reg
//
// An extra function to write a single register.
// It is just a wrapper around the MPU_6050_write()
// function, and it is only a convenient function
// to make it easier to write a single register.
//
int IMU::MPU6050_write_reg(int reg, uint8_t data)
{
  int error;

  error = MPU6050_write(reg, &data, 1);

  return (error);
}


// -----------------------------------------
//---------------
// MPU6050_read
//
// This is a common function to read multiple bytes 
// from an I2C device.
//
// It uses the boolean parameter for Wire.endTransMission()
// to be able to hold or release the I2C-bus. 
// This is implemented in Arduino 1.0.1.
//
// Only this function is used to read. 
// There is no function for a single byte.
//

int IMU::MPU6050_read(int start, uint8_t *buffer, int size)
{
  int i, n, error;

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);
  if (n != 1)
    return (-10);

  n = Wire.endTransmission(false);    // hold the I2C-bus
  if (n != 0)
    return (n);

  // Third parameter is true: relase I2C-bus after data is read.
  Wire.requestFrom(MPU6050_I2C_ADDRESS, size, true);
  i = 0;
  while(Wire.available() && i<size)
  {
    buffer[i++]=Wire.read();
  }
  if ( i != size)
    return (-11);

  return (0);  // return : no error
}




/*****************************************
complementary filter based on information
from http://robottini.altervista.org/kalman-filter-vs-complementary-filter/filters1
******************************************/
void IMU::getCurAngle(float &Xangle, float &XAngularVelocity)
{
	updateValues();
	int curtime = micros();
	int dtime = curtime - lastUpdate;
	lastUpdate = curtime;

	//calculate angle based on accelerometer
	int x_accel = lastSensorValues.value.z_accel;
	int y_accel = lastSensorValues.value.y_accel;

	float accelAngle = atan2(y_accel,x_accel)*57.2957795;

	//get the angular velocity
	XAngularVelocity = lastSensorValues.value.x_gyro / 16.4;



	float A = 0.075/ (0.075 + dtime);


	Xangle = angle = A*(angle + XAngularVelocity * dtime) + (1-A) * (accelAngle);
}

void IMU::getCurAngleKalman(float &Xangle, float &XAngularVelocity, float &accelAngle)
{
	updateValues();
	int curtime = micros();
	int dtime = curtime - lastUpdate;
	lastUpdate = curtime;

	//calculate angle based on accelerometer
	int x_accel = lastSensorValues.value.z_accel;
	int y_accel = lastSensorValues.value.y_accel;

	accelAngle = atan2(y_accel,x_accel)*57.2957795;

	//get the angular velocity
	XAngularVelocity = lastSensorValues.value.x_gyro / 16.4;
	
	Xangle = filter.getAngle(accelAngle,XAngularVelocity, dtime);
}

void IMU::getPitchAndRate(float &accelAngle, float &gyroate)
{
	updateValues();
	int x_accel = lastSensorValues.value.z_accel;
	int y_accel = lastSensorValues.value.y_accel;

	accelAngle = atan2(y_accel,x_accel)*57.2957795;
	
	gyroate = ((float)lastSensorValues.value.x_gyro) / 16.4;
}





