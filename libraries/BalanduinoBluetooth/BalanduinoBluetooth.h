#ifndef BTOOTH_H_
#define BTOOTH_H_
#include <SoftwareSerial.h>
#include <Arduino.h>
class BalanduinoBluetooth
{
private:
	bool readImu;
	bool readJoystick;
	bool sendGraph;
	bool sendPID;
	bool readPID;
	bool getInfo;
	bool steerStop;
	long lastTransmission;
	void resetAllBool();

	SoftwareSerial* ss;//ss(12,13);

	float speed;
	float direction;
	float Kp;
	float Ki;
	float Kd;
	float target;


public:
	//starts the Serial with tx,rx pins as input 
	//make sure the pins sent in are compatible with software serial
	//for the uno it is 	
	BalanduinoBluetooth();
	void updateValues();	

	// uint8_t read(){return ss.read();};
	// bool available(){return ss.available();};
	float getKi(){return Ki;};
	float getKp(){return Kp;};
	float getKd(){return Kd;};
	float getSpeed(){return speed;};
	float getDirection(){return direction;};
	float getTargets(){return target;};
	void printGraph(float,float,float);
	bool getGraphBool(){return sendGraph;};
};

#endif