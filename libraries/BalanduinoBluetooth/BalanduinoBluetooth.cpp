#include "BalanduinoBluetooth.h"
#include <Arduino.h>
#include <SoftwareSerial>

BalanduinoBluetooth::BalanduinoBluetooth()
{
	ss = new SoftwareSerial(A1,A0);
	ss->begin(57600);
	lastTransmission = millis();
	target = 180;
}


void BalanduinoBluetooth::resetAllBool()
{
	readImu = false;
	readJoystick = false;
	sendGraph = false;
	sendPID = false;
	readPID = false;
	getInfo = false;
	steerStop = false;
}




void BalanduinoBluetooth::updateValues()
{
	if(ss->available()>2)
	{
		char input[30];
		short i =0;

		//read until you reach a semi colon
		while(1)
		{
			input[i] = ss->read();
			if(input[i]==-1)
			{
				Serial.println("Error");
				return; //error reading
			}
			if(input[i] == ';')
			{
				break; //break on semicolons
			}
			++i;
		}
		 if(input[0] == 'C'  ) //recieved a Controll command
		 {
			if(input[1] == 'S')
			{
				steerStop = true;
				// Serial.println("stopping");
				speed = 0; 
				direction = 0; 
			}
			else if(input[1]=='M')
			{
				readImu = true;
				strtok( input,",");
				// Serial.println(strtok( input,",")); //get rid of the CM,
				 direction = atof(strtok(NULL,","));
				 speed = atof(strtok(NULL,","));

			  	// Serial.println(direction);
			  	// Serial.println(speed);

		 	}
		 	else if (input[1] == 'J')
			{
				readJoystick = true;
				strtok( input,",");
				// Serial.println(strtok( input,",")); //get rid of the CM,
				 direction = atof(strtok(NULL,","));
				 speed = atof(strtok(NULL,","));

			  	// Serial.println(direction);
			  	// Serial.println(speed);
			}
		}
		else if(input[0] == 'S')
		{
			readPID = true;
			if(input[1] == 'P')
			{

				 
				strtok( input,","); //get rid of the CM,
				float t = atof(strtok(NULL,","));				 
				if(((int)t )!= 0)
				 Kp = t;
			}
			else if(input[1] == 'D')
			{

				 
				 strtok( input,","); //get rid of the CM,
				 float t = atof(strtok(NULL,","));
				 if(((int)t )!= 0)
				 Kd = t;
			}
			else if(input[1] == 'I')
			{
		 
				 strtok( input,","); //get rid of the CM,
				 float t = atof(strtok(NULL,","));
				 if(((int)t )!= 0)
				 Ki = t;
			}
			else if(input[1] == 'T')
			{

				 strtok( input,","); //get rid of the CM,
				 float t = atof(strtok(NULL,","));
				 if(((int)t )!= 0)
				 	 target =t;
			}
		}
		else if(input[0] == 'I') //graph screen
		{
			if(input[1]=='S')
			{
				sendGraph = false;
			}
			else
			{
				sendGraph = true;
			}
		}
		// else if(input[0]== 'G' && (millis() - lastTransmission) > 50)
		// {
		// 	Serial.println("transmitting");
		// 	lastTransmission = millis();
		// 	if(input[1]== 'P')
		// 	{
		// 		  ss->print("P,");
		// 	      ss->print(Kp);
		// 	      ss->print(',');
		// 	      ss->print(Ki);
		// 	      ss->print(',');
		// 	      ss->print(Kd);
		// 	      ss->print(',');
		// 	      ss->println(target);
		// 	}
		// }
	}
}


void BalanduinoBluetooth::printGraph(float accelAngle, float gyro, float angle)
{
      ss->print("V,");
      ss->print(accelAngle+180);
      ss->print(',');
      ss->print(gyro+180);
      ss->print(',');
      ss->println(angle+180);	
}


// void BalanduinoBluetooth::updateValues()
// {
// 	Serial.println("starting");
// 	if(ss->available())
// 	{

// 		Serial.println("reading");
// 		char input[30];
// 		uint8_t i =0;

// 		//read until you reach a semi colon
// 		while(input[i])
// 		{
// 			input[i] = ss->read();
// 			if(input[i]==-1)
// 			{
// 				return; //error reading
// 			}
// 			if(input[i] == ';')
// 			{
// 				break; //break on semicolons
// 			}
// 			++i;
// 		}
// 		Serial.println(input);
// 		if(input[0] == 'C'  ) //recieved a Controll command
// 		{
// 			if(input[1] == 'S')
// 			{
// 				steerStop = true;
// 			}
// 			else if(input[1]=='M')
// 			{
// 				readImu = true;
// 				strtok( input,","); //get rid of the CM,
// 				direction = atof(strtok(NULL,","));
// 				speed = atof(strtok(NULL,","));
// 			}
// 			else if (input[1] == 'J')
// 			{
// 				readJoystick = true;

// 				direction = atof(strtok(NULL,","));
// 				speed = atof(strtok(NULL,","));			
// 			}
// 		}
// 		else if(input[0] == 'I') // Graphs
// 		{
// 			if(input[1] == 'S')
// 			{
// 				sendGraph == false;
// 			}
// 			else 
// 			{
// 				sendGraph == true;
// 			}
// 		}
// 		else if(input[0] == 'S') // PID adjustment
// 		{
// 			if(input[1]=='I')
// 			{
// 				strtok( input,",");
// 				Ki = atof(strtok(NULL,","));	
// 			}
// 			else if(input[1]=='D')
// 			{
// 				strtok( input,",");
// 				Kd = atof(strtok(NULL,","));	
// 			}
// 			else if(input[1]=='T')
// 			{
// 				strtok( input,",");
// 				Kp = atof(strtok(NULL,","));	
// 			}
// 		}

// 	}
// 	Serial.println("done");
// }

