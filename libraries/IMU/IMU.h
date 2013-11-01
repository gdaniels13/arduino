
#ifndef IMU_H_
#define IMU_H_

#include <Wire.h>
#include "Kalman.h"
#include <stdint.h> // Needed for uint8_t
#include <Arduino.h>

#define MPU6050_I2C_ADDRESS 0x68
#define MPU6050_PWR_MGMT_1         0x6B   // R/W
#define MPU6050_ACCEL_XOUT_H       0x3B   // R  


#define gyroRate accel_t_gyro.value.x_gyro
#define SWAP(x,y) swap = x; x = y; y = swap
#define PI 3.14159265358979f




typedef union accel_t_gyro_union
{
  struct
  {
    uint8_t x_accel_h;
    uint8_t x_accel_l;
    uint8_t y_accel_h;
    uint8_t y_accel_l;
    uint8_t z_accel_h;
    uint8_t z_accel_l;
    uint8_t t_h;
    uint8_t t_l;
    uint8_t x_gyro_h;
    uint8_t x_gyro_l;
    uint8_t y_gyro_h;
    uint8_t y_gyro_l;
    uint8_t z_gyro_h;
    uint8_t z_gyro_l;
  } reg;
  struct 
  {
    int x_accel;
    int y_accel;
    int z_accel;
    int temperature;
    int x_gyro;
    int y_gyro;
    int z_gyro;
  } value;
};


class IMU
{
  public:
  	IMU();
  	void Calibrate();
  	void GetRawAccel(int& x, int& y, int & z);
  	void GetRawGyro(int& x, int& y, int & z);
    void getCurAngle(float &Xangle, float &XAngularVelocity);
	  void getCurAngleKalman(float &Xangle, float &XAngularVelocity, float &accelAngle);
  	void getPitchAndRate(float&,float&);
	
	
  private:
    int MPU6050_write_reg(int reg, uint8_t data);
    int MPU6050_write(int start, const uint8_t *pData, int size);
    int MPU6050_read(int start, uint8_t *buffer, int size);

    void updateValues();

    accel_t_gyro_union lastSensorValues;
  	float gyroOffset,accelxoffset,accelyoffset,accelzoffset;
  	float angle;
    int lastUpdate;
	Kalman filter;
	
};

#endif
