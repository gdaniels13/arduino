#include <Arduino.h>
#include <Motor.h>
#include <Encoder.h>

#define SET_GOAL 'a'
#define SET_LEFT 1
#define SET_RIGHT 2
#define RESET_ENCODER 3
#define DRIVE_LEFT 4
#define DRIVE_RIGHT 5
#define PADDLE_FORWARD 6
#define PADDLE_BACKWARD 7
#define AUTOMATIC 9


#define leftButtonVal digitalRead(A1)
#define rightButtonVal digitalRead(A0)

Motor driveMotor(5,7,8);
Motor paddleMotor(6,9,10);
Encoder encoder(3,2);


int MotorStartPoint = 80;
int goal=25;	
int goalPercent = 50;					// goal position;
int left = 0;
int right = 50;
boolean mode = true;
void setup() 
{
	Serial.begin(115200);
	calibrate();

}


void calibrate(){
	InitialiseIO();
	//first Calibrate

	moveLeft(200);
	while(leftButtonVal==1);
	driveMotor.stop();
	encoder.write(0);
	left = 0;
	Serial.println("found far left");
	moveRight(200);
	while(rightButtonVal == 1);
	driveMotor.stop();
	right = encoder.read() +0;	
	serialPrint("found right at : ",right);
	goal = calculateGoal();
	InitialiseInterrupt();


}

void InitialiseIO(){
  pinMode(A0, INPUT);	   // Pin A0 is input to which a switch is connected
  digitalWrite(A0, HIGH);   // Configure internal pull-up resistor
  pinMode(A1, INPUT);	   // Pin A1 is input to which a switch is connected
  digitalWrite(A1, HIGH);   // Configure internal pull-up resistor
}

void InitialiseInterrupt(){
  cli();		// switch interrupts off while messing with their settings  
  PCICR =0x02;          // Enable PCINT1 interrupt
  PCMSK1 = 0b00000111;
  sei();		// turn interrupts back on
}

ISR(PCINT1_vect) {    // Interrupt service routine. Every single PCINT8..14 (=ADC0..5) change
            // will generate an interrupt: but this will always be the same interrupt routine
  if (digitalRead(A0)==0) {//right
  	right = encoder.read();
  }
  else if (digitalRead(A1)==0){//left
  	encoder.write(0);
  	left = 0;
  }
  driveMotor.stop();
}

void loop()
{
	readSerial();
	// Serial.println(encoder.read());
	if(mode){
		int curPos = encoder.read() - goal;
		bool forward = false;
		int power = 0;
		if(curPos<0){
			forward = true;
			curPos *=-1;
		}
		if(curPos<3){
			power = 0;
		}
		else if (curPos<10){
			// forward = !forward;
			power = 180;
		}
		else if (curPos < 20){
			power = 235;
		}
		else {
			power = 255;
		}
		if(forward){
			driveMotor.forward(power);
		}
		else{
			driveMotor.backward(power);
		}
	}
}

void readSerial(){
	if(Serial.available()<2){

		return; // nothing to do here
	}
	int a = 0;
	byte command = Serial.read();
	byte data = Serial.read();
	while(Serial.available()){
		Serial.read();
	}
	switch (command) {
	    case SET_GOAL:
	    	goalPercent = data;
	    	goal = calculateGoal();
	    	break;
	    case SET_LEFT:
	    	left = data;
	    	goal = calculateGoal();
	    	break;
	    case SET_RIGHT:
	    	right = data;
	    	goal = calculateGoal();
	    	break;
	    case RESET_ENCODER:
	    	encoder.write(0);
	    break;
		case DRIVE_LEFT:
			moveLeft(data);
		break;
		case DRIVE_RIGHT:
			moveRight(data);
		break;
		case PADDLE_FORWARD:
			paddleMotor.forward(data);
		break;
		case PADDLE_BACKWARD:
			 goalPercent = 50;
			paddleMotor.backward(data);
		break;
		case AUTOMATIC:
			mode = data;
		break;
		};
}

void moveLeft(byte b){
	driveMotor.backward(b);
}

void moveRight(byte b){
	driveMotor.forward(b);
}

int calculateGoal(){
	return (right -left) * goalPercent / 100.0;
}
void serialPrint(char* s,int t){
	Serial.print(" ");
	Serial.print(s);
	Serial.print(t);
}

void serialPrint(char* s,double t){
	Serial.print(" ");
	Serial.print(s);
	Serial.print(t);
}

void serialPrint(char* s,char*t){
	Serial.print(" ");
	Serial.print(s);
	Serial.print(t);
}