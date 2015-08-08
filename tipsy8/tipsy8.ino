// tipsy8.ino

#include <Wire.h>
#include <I2Cdev.h>
#include <JJ_MPU6050_DMP_6Axis.h>  // Modified version of the library to work with DMP (see comments inside)
#include <BalanduinoBluetooth.h>
#include <SoftwareSerial.h>
#include <string>


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

//constants
#define DEBUG 0
#define SHUTDOWN_WHEN_BATTERY_OFF 1

#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))

#define ZERO_SPEED 65535
#define MAX_ACCEL 7

#define MAX_THROTTLE 400
#define MAX_STEERING 100
#define MAX_TARGET_ANGLE 12

// PRO MODE = MORE AGGRESSIVE
#define MAX_THROTTLE_PRO 650
#define MAX_STEERING_PRO 240 
#define MAX_TARGET_ANGLE_PRO 18


// #define I2C_SPEED 100000L
#define I2C_SPEED 400000L
//#define I2C_SPEED 800000L

#define ACCEL_SCALE_G 8192             // (2G range) G = 8192
#define ACCEL_WEIGHT 0.01
#define GYRO_BIAS_WEIGHT 0.005

// MPU6000 sensibility   (0.0609 => 1/16.4LSB/deg/s at 2000deg/s, 0.03048 1/32.8LSB/deg/s at 1000deg/s)
#define Gyro_Gain 0.03048
#define Gyro_Scaled(x) x*Gyro_Gain //Return the scaled gyro raw data in degrees per second

#define RAD2GRAD 57.2957795
#define GRAD2RAD 0.01745329251994329576923690768489

// Default control terms   
#define KP 0.20 // 0.22        
#define KD 26   // 30 28        
#define KP_THROTTLE 0.065  //0.08
#define KI_THROTTLE 0.05

#define ITERM_MAX_ERROR 40   // Iterm windup constants
#define ITERM_MAX 5000
#define DEADBAND 0.8


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (for us 18 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[18]; // FIFO storage buffer
Quaternion q;

uint8_t loop_counter;       // To generate a medium loop 40Hz 
uint8_t slow_loop_counter;  // slow loop 2Hz
long timer_old;
long timer_value;
int debug_counter;
float dt;

MPU6050 mpu;
BalanduinoBluetooth blue;

float angle_adjusted;
float angle_adjusted_Old;

float Kp=KP;
float Kd=KD;
float Kp_thr=KP_THROTTLE;
float Ki_thr=KI_THROTTLE;
float Kp_user=KP;
float Kd_user=KD;
float Kp_thr_user=KP_THROTTLE;
float Ki_thr_user=KI_THROTTLE;
bool newControlParameters = false;
bool modifing_control_parameters=false;
float PID_errorSum;
float PID_errorOld = 0;
float PID_errorOld2 = 0;
float setPointOld = 0;
float target_angle;
float throttle;
float steering;
float max_throttle = MAX_THROTTLE;
float max_steering = MAX_STEERING;
float max_target_angle = MAX_TARGET_ANGLE;
float control_output;
int16_t motor1;
int16_t motor2;

int16_t speed_m[2];           // Actual speed of motors
uint8_t dir_m[2];             // Actual direction of steppers motors
int16_t actual_robot_speed;          // overall robot speed (measured from steppers speed)
int16_t actual_robot_speed_Old;          // overall robot speed (measured from steppers speed)
float estimated_speed_filtered;

volatile uint16_t counter_m[2];        // counters for periods
volatile uint16_t period_m[2][8];      // Eight subperiods 
volatile uint8_t period_m_index[2];    // index for subperiods


ISR(TIMER1_COMPA_vect)
// void motorControllISR()
{
	counter_m[LEFT_MOTOR]++;
	counter_m[1]++;
	if (counter_m[LEFT_MOTOR] >= period_m[LEFT_MOTOR][period_m_index[0]]){
		counter_m[LEFT_MOTOR] = 0;
		if (period_m[0][0]==ZERO_SPEED)
			return;
		if (dir_m[0])
			SET(MOTOR_PORT,LEFT_MOTOR_DIR);  // DIR Motor 1
		else
			CLR(MOTOR_PORT,LEFT_MOTOR_DIR);
		// We need to wait at lest 200ns to generate the Step pulse...
		period_m_index[0] = (period_m_index[0]+1)&0x07; // period_m_index from 0 to 7
		//delay_200ns();
		SET(MOTOR_PORT,LEFT_MOTOR_STEP); // STEP Motor 1
		delayMicroseconds(1);
		CLR(MOTOR_PORT,LEFT_MOTOR_STEP);
	}
	if (counter_m[1] >= period_m[1][period_m_index[1]]){
		counter_m[1] = 0;
		if (period_m[1][0]==ZERO_SPEED)
			return;
		if (dir_m[1])
			SET(MOTOR_PORT,RIGHT_MOTOR_DIR);   // DIR Motor 2
		else
			CLR(MOTOR_PORT,RIGHT_MOTOR_DIR);
		period_m_index[1] = (period_m_index[1]+1)&0x07;
		//delay_200ns();
		SET(MOTOR_PORT,RIGHT_MOTOR_STEP); // STEP Motor 1
		delayMicroseconds(1);
		CLR(MOTOR_PORT,RIGHT_MOTOR_STEP);
	}
}


// DMP FUNCTIONS
// This function defines the weight of the accel on the sensor fusion
// default value is 0x80
// The official invensense name is inv_key_0_96 (??)
void dmpSetSensorFusionAccelGain(uint8_t gain)
{
  // INV_KEY_0_96
  mpu.setMemoryBank(0);
  mpu.setMemoryStartAddress(0x60);
  mpu.writeMemoryByte(0);
  mpu.writeMemoryByte(gain);
  mpu.writeMemoryByte(0);
  mpu.writeMemoryByte(0);
}

int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}


// Quick calculation to obtein Phi angle from quaternion solution
float dmpGetPhi() {
   mpu.getFIFOBytes(fifoBuffer, 16); // We only read the quaternion
   mpu.dmpGetQuaternion(&q, fifoBuffer); 
   mpu.resetFIFO();  // We always reset FIFO
    
   //return( asin(-2*(q.x * q.z - q.w * q.y)) * 180/M_PI); //roll
   return (atan2(2*(q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z)* RAD2GRAD);
}

// PD implementation. DT is in miliseconds
float stabilityPDControl(float DT, float input, float setPoint,  float Kp, float Kd)
{
  float error;
  float output;

  error = setPoint-input;

  // Kd is implemented in two parts
  //    The biggest one using only the input (sensor) part not the SetPoint input-input(t-2)
  //    And the second using the setpoint to make it a bit more agressive   setPoint-setPoint(t-1)
  output = Kp*error + (Kd*(setPoint - setPointOld) - Kd*(input - PID_errorOld2))/DT;       // + error - PID_error_Old2
  //Serial.print(Kd*(error-PID_errorOld));Serial.print("\t");
  PID_errorOld2 = PID_errorOld;
  PID_errorOld = input;  // error for Kd is only the input component
  setPointOld = setPoint;
  return(output);
}

// P control implementation.
float speedPControl(float input, float setPoint,  float Kp)
{
  float error;

  error = setPoint-input;
 
  return(Kp*error);
}

// PI implementation. DT is in miliseconds
float speedPIControl(float DT, float input, float setPoint,  float Kp, float Ki)
{
  float error;
  float output;

  error = setPoint-input;
  PID_errorSum += constrain(error,-ITERM_MAX_ERROR,ITERM_MAX_ERROR);
  PID_errorSum = constrain(PID_errorSum,-ITERM_MAX,ITERM_MAX);

  output = Kp*error + Ki*PID_errorSum*DT*0.001;
  return(output);
}

// 200ns => 4 instructions at 16Mhz
void delay_200ns()  
{
  __asm__ __volatile__ (
		"nop" "\n\t"
                "nop" "\n\t"
                "nop" "\n\t"
		"nop"); 
}


// Dividimos en 8 subperiodos para aumentar la resolucion a velocidades altas (periodos pequeños)
// subperiod = ((1000 % vel)*8)/vel;
// Examples 4 subperiods:
// 1000/260 = 3.84  subperiod = 3
// 1000/240 = 4.16  subperiod = 0
// 1000/220 = 4.54  subperiod = 2
// 1000/300 = 3.33  subperiod = 1 
void calculateSubperiods(uint8_t motor)
{
  int subperiod;
  int absSpeed;
  uint8_t j;
  
  if (speed_m[motor] == 0)
    {
    for (j=0;j<8;j++)
      period_m[motor][j] = ZERO_SPEED;
    return;
    }
  if (speed_m[motor] > 0 )   // Positive speed
    {
    dir_m[motor] = 1;
    absSpeed = speed_m[motor];
    }
  else                       // Negative speed
    {
    dir_m[motor] = 0;
    absSpeed = -speed_m[motor];
    }
    
  for (j=0;j<8;j++)
    period_m[motor][j] = 1000/absSpeed;
  // Calculate the subperiod. if module <0.25 => subperiod=0, if module < 0.5 => subperiod=1. if module < 0.75 subperiod=2 else subperiod=3
  subperiod = ((1000 % absSpeed)*8)/absSpeed;   // Optimized code to calculate subperiod (integer math)
  if (subperiod>0)
   period_m[motor][1]++;
  if (subperiod>1)
   period_m[motor][5]++;
  if (subperiod>2)
   period_m[motor][3]++;
  if (subperiod>3)
   period_m[motor][7]++;
  if (subperiod>4)
   period_m[motor][0]++;
  if (subperiod>5)
   period_m[motor][4]++;
  if (subperiod>6)
   period_m[motor][2]++;
}

void setMotorSpeed(uint8_t motor, int16_t tspeed)
{
  // WE LIMIT MAX ACCELERATION
  if ((speed_m[motor] - tspeed)>MAX_ACCEL)
    speed_m[motor] -= MAX_ACCEL;
  else if ((speed_m[motor] - tspeed)<-MAX_ACCEL)
    speed_m[motor] += MAX_ACCEL;
  else
    speed_m[motor] = tspeed;
  
  calculateSubperiods(motor);  // We use four subperiods to increase resolution
  
  // To save energy when its not running...
  if ((speed_m[0]==0)&&(speed_m[1]==0)){
    digitalWrite(MOTOR_ENABLE,HIGH);   // Disable motors
  }
  else{
    digitalWrite(MOTOR_ENABLE,LOW);   // Enable motors
  }
}

void setup(){
	Serial.begin(9600);
	Serial.println("starting setups");
	Serial.println("enabling motor and Timer1");
	motorInit();
	timer1_init();

	Serial.println("enabling I2C");
//set up the IMU on the I2C bus
	Wire.begin();
	//I dont know what this does but will probably work
	//verified IMU works
	TWSR = 0;
  	TWBR = ((16000000L/I2C_SPEED)-16)/2;
  	TWCR = 1<<TWEN;


	devStatus = mpu.dmpInitialize();
	if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
    } else { // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
        while(1);
    }
  
  // Gyro calibration
  // The robot must be steady during initialization
  	for(int i=0; i<10; i++){
  		Serial.println(i);
  		delay(1000);
  	}

	// freeRam();
    //Adjust sensor fusion gain
    dmpSetSensorFusionAccelGain(0x20);
  //what does this do??
    delay(2000);
      	for (uint8_t k=0;k<3;k++)
	{
		setMotorSpeed(LEFT_MOTOR,3);
		setMotorSpeed(RIGHT_MOTOR,-3);
		delay(150);
		setMotorSpeed(LEFT_MOTOR,-3);
		setMotorSpeed(RIGHT_MOTOR,3);
		delay(150);
	}

	mpu.resetFIFO();
  	timer_old = millis();


}

// void loop(){
// 	Serial.println(	angle_adjusted = dmpGetPhi());
	
// }

void motorInit(){
	pinMode(MS1, OUTPUT);	
	pinMode(MS2, OUTPUT);	
	pinMode(MS3, OUTPUT);	
	pinMode(LEFT_MOTOR_STEP, OUTPUT);	
	pinMode(LEFT_MOTOR_DIR, OUTPUT);	
	pinMode(MOTOR_ENABLE, OUTPUT);	
	pinMode(RIGHT_MOTOR_STEP, OUTPUT);	
	pinMode(RIGHT_MOTOR_DIR, OUTPUT);	
	digitalWrite(MS1, HIGH);
	digitalWrite(MS2, HIGH);
	digitalWrite(MS3, LOW);
	setMotorSpeed(LEFT_MOTOR,0);
	setMotorSpeed(RIGHT_MOTOR,0);
}

void timer1_init()
{
  cli();
    // set up timer with prescaler = 8 and CTC mode
    // initialize counter
    TCCR1A = 0;
    TCCR1B = 0;

    TCCR1B |= (1 << WGM12)|(0 << CS12)|(0 << CS11)|(1 << CS10);
  
  
    // initialize compare value
  
    // OCR1A = 15624;
    OCR1A = 640;
    // enable compare interrupt
    TIMSK1 |= (1 << OCIE1A);
  
    // enable global interrupts
    sei();
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


// Main loop
void loop() 
{ 
	blue.updateValues();

	throttle =blue.getSpeed();
	steering = -blue.getDirection();

	if(throttle<0){
		throttle =fscale(-1,0,-MAX_THROTTLE,0,throttle,-2);
	}
	else if(throttle>0){
		throttle =fscale(0,1,0,MAX_THROTTLE,throttle,-2);
	}
	if(steering<0){
		steering =fscale(-1,0,-MAX_STEERING,0,steering,-2);
	}
	else if (steering>0){
		steering =fscale(0,1,0,MAX_STEERING,steering,-2);
	}

	// Serial.print(throttle);
	// Serial.print(" : ");
	// Serial.println(steering);
		 
	timer_value = millis();
	// New DMP Orientation solution?
	fifoCount = mpu.getFIFOCount();
	if (fifoCount>=18){
		if (fifoCount>18)  // If we have more than one packet we take the easy path: discard the buffer 
		{
			Serial.println("FIFO RESET!!");
			mpu.resetFIFO();
			return;
		}
		loop_counter++;
		slow_loop_counter++;
		dt = (timer_value-timer_old);
		timer_old = timer_value;
		angle_adjusted_Old = angle_adjusted;
		angle_adjusted = dmpGetPhi();
		// Serial.println(angle_adjusted);
		mpu.resetFIFO();  // We always reset FIFO
		//read 

		// We calculate the estimated robot speed
		// Speed = angular_velocity_of_stepper_motors - angular_velocity_of_robot(angle measured by IMU)
		actual_robot_speed_Old = actual_robot_speed;
		actual_robot_speed = (speed_m[1] - speed_m[0])/2;  // Positive: forward

		int16_t angular_velocity = (angle_adjusted-angle_adjusted_Old)*90.0;     // 90 is an empirical extracted factor to adjust for real units
		int16_t estimated_speed = actual_robot_speed_Old - angular_velocity;     // We use robot_speed(t-1) or (t-2) to compensate the delay
		estimated_speed_filtered = estimated_speed_filtered*0.95 + (float)estimated_speed*0.05;
		// #if DEBUG==2
		// 	Serial.print(" ");
		// 	Serial.println(estimated_speed_filtered);
		// #endif
		
		target_angle = speedPIControl(dt,estimated_speed_filtered,throttle,Kp_thr,Ki_thr); 
		target_angle = constrain(target_angle,-max_target_angle,max_target_angle);   // limited output
		target_angle = 180-blue.getTargets() + target_angle;
		// #if DEBUG==3
			// Serial.print(" ");Serial.println(estimated_speed_filtered);
			// Serial.println("target_angle ");Serial.println(target_angle);
		// #endif
		
		// New user control parameters ?
		//TODO:  need to implement this with USB
		// readControlParameters(); 
	
		// We integrate the output (acceleration)
		control_output += stabilityPDControl(dt,angle_adjusted,target_angle,Kp,Kd);	
		control_output = constrain(control_output,-500,500);   // Limit max output from control
		
				
		// The steering part of the control is injected directly on the output
		motor1 = control_output + steering;
		motor2 = -control_output + steering;   // Motor 2 is inverted
		
	// Limit max speed
		motor1 = constrain(motor1,-500,500);   
		motor2 = constrain(motor2,-500,500);
		
	// Is robot ready (upright?)
		if ((angle_adjusted<74)&&(angle_adjusted>-74)){
			   // NORMAL MODE
				setMotorSpeed(LEFT_MOTOR,motor1);
				setMotorSpeed(RIGHT_MOTOR,motor2);
		}
			
		if ((angle_adjusted<40)&&(angle_adjusted>-40))
		{
			Kp = Kp_user;  // Default or user control gains
			Kd = Kd_user; 
			Kp_thr = Kp_thr_user;
			Ki_thr = Ki_thr_user;
		} else {   // Robot not ready, angle > 70º
			setMotorSpeed(LEFT_MOTOR,0);
			setMotorSpeed(RIGHT_MOTOR,0);
			PID_errorSum = 0;  // Reset PID I term
		} // New IMU data
	
	// if (slow_loop_counter>=99)  // 2Hz
	// 	{
	// 	slow_loop_counter = 0;
	// 	// Read battery status
		// #if DEBUG==7
		// 	Serial.print(distance_sensor);
		// 	Serial.print(" A");
		// 	Serial.println(autonomous_mode_status);
		// #endif
		// if (battery < BATTERY_SHUTDOWN)
		// 	{
		// 	// Robot shutdown !!!
		// 	#ifdef SHUTDOWN_WHEN_BATTERY_OFF
		// 		Serial.println("LOW BAT!! SHUTDOWN"); 
		// 		Robot_shutdown = true;
		// 		// Disable steppers
		// 		digitalWrite(4,HIGH);   // Disable motors
		// 	#endif	
		// 	}
		// else if (battery < BATTERY_WARNING)
		// 	{
		// 	// Battery warning
		// 	// What to do here???
		// 	Serial.println("LOW BAT!!");
		// 	WITA.Servo(3,SERVO_AUX_NEUTRO+300);  // Move arm?
		// 	}
		// }  // Slow loop
}}


float fscale( float originalMin, float originalMax, float newBegin, float newEnd, float inputValue, float curve){

  float OriginalRange = 0;
  float NewRange = 0;
  float zeroRefCurVal = 0;
  float normalizedCurVal = 0;
  float rangedValue = 0;
  boolean invFlag = 0;


  // condition curve parameter
  // limit range

  if (curve > 10) curve = 10;
  if (curve < -10) curve = -10;

  curve = (curve * -.1) ; // - invert and scale - this seems more intuitive - postive numbers give more weight to high end on output
  curve = pow(10, curve); // convert linear scale into lograthimic exponent for other pow function

  /*
   Serial.println(curve * 100, DEC);   // multply by 100 to preserve resolution  
   Serial.println();
   */

  // Check for out of range inputValues
  if (inputValue < originalMin) {
    inputValue = originalMin;
  }
  if (inputValue > originalMax) {
    inputValue = originalMax;
  }

  // Zero Refference the values
  OriginalRange = originalMax - originalMin;

  if (newEnd > newBegin){
    NewRange = newEnd - newBegin;
  }
  else
  {
    NewRange = newBegin - newEnd;
    invFlag = 1;
  }

  zeroRefCurVal = inputValue - originalMin;
  normalizedCurVal  =  zeroRefCurVal / OriginalRange;   // normalize to 0 - 1 float

  /*
  Serial.print(OriginalRange, DEC);  
   Serial.print("   ");  
   Serial.print(NewRange, DEC);  
   Serial.print("   ");  
   Serial.println(zeroRefCurVal, DEC);  
   Serial.println();  
   */

  // Check for originalMin > originalMax  - the math for all other cases i.e. negative numbers seems to work out fine
  if (originalMin > originalMax ) {
    return 0;
  }

  if (invFlag == 0){
    rangedValue =  (pow(normalizedCurVal, curve) * NewRange) + newBegin;

  }
  else     // invert the ranges
  {  
    rangedValue =  newBegin - (pow(normalizedCurVal, curve) * NewRange);
  }

  return rangedValue;
}