// lineFollower.ino
#include <QTRSensors.h>
#include <SoftPWM.h>
#include <Motor.h>


#define NUM_SENSORS   6     // number of sensors used
#define TIMEOUT       4000  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2
#define MAX_SPEED 130
#define MIN_SPEED 0

#define THRESH 500
#define GOAL 2500
Motor leftMotor(3,4,5);
Motor rightMotor(6,7,8);

QTRSensorsRC sensors((unsigned char[]) {A0,A1,A2,A3,A4,A5}, NUM_SENSORS,TIMEOUT); 
unsigned int sensorValues[NUM_SENSORS];
unsigned int values[NUM_SENSORS];
void setup() {
	Serial.begin(9600);

  Serial.begin(9600);
  Serial.println("beginning calibration");
  delay(500);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 100; i++)  // make the calibration take about 10 seconds
  {
    sensors.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }
  digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration


}

void go(int a,int b){
	leftMotor.forward(a);
	rightMotor.forward(b);
}

void loop() {
  int pos = whereAmI();
  Serial.println(pos);
  int error = pos -GOAL ;
  int mag = abs(error);

  if(error<0){ // go right
  	if(mag<100)
  		go(MAX_SPEED,MAX_SPEED);
  	else if(mag<200)
  		go(MAX_SPEED,MAX_SPEED-10);
  	else if(mag<500)
  		go(MAX_SPEED,MAX_SPEED-20);
	else if(mag<200)
  		go(MAX_SPEED,MAX_SPEED-30);
  	else if(mag<200)
  		go(MAX_SPEED,MAX_SPEED-50);
  	else if(mag<200)
  		go(MAX_SPEED,MAX_SPEED-70);

  } else { // go left;

	if(mag<100)
  		go(MAX_SPEED,MAX_SPEED);
  	else if(mag<200)
  		go(MAX_SPEED-10,MAX_SPEED);
  	else if(mag<500)
  		go(MAX_SPEED-20,MAX_SPEED);
	else if(mag<200)
  		go(MAX_SPEED-30,MAX_SPEED);
  	else if(mag<200)
  		go(MAX_SPEED-50,MAX_SPEED);
  	else if(mag<200)
  		go(MAX_SPEED-70,MAX_SPEED);

  }


  // leftMotor.forward(leftSpeed);
  // rightMotor.forward(rightSpeed);

  
}

//-left
//0 dead center
//+ right

int whereAmI(){
	int value = sensors.readLine(values);
	bool bools[NUM_SENSORS];
	for(int i=0; i<NUM_SENSORS; i++){
	    bools[i] = values[i]<THRESH;
	}
	// printArray(values);
	return value;

}

void printArray(bool* array){
	for(int i = 0; i< NUM_SENSORS; ++i){
  	Serial.print(array[i]);
  	Serial.print('\t');
  }
  Serial.println();	
}

bool arrayEquals(bool* array1,bool*array2){
	for(int i=0; i<NUM_SENSORS; i++){
	    if(array1[i]!=array2[i]){
	    	return false;
	    }
	}
	return true;
}


void printArray(unsigned int* array){
	for(int i = 0; i< NUM_SENSORS; ++i){
  	Serial.print(array[i]);
  	Serial.print('\t');
  }
  Serial.println();	
}

int addArrays(){
	for(int i=0; i<NUM_SENSORS; i++){
	    values[i]+=sensorValues[i];
	}
}