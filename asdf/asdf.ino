#include <BalanduinoBluetooth.h>
 #include <SoftwareSerial.h>
 #include <string>

int x;

#define ZERO_SPEED 65535
#define MAX_ACCEL 50
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

#define SET(x,y) (x|=(1<<y))
#define CLR(x,y) (x&=(~(1<<y)))


#define ISR_FREQUENCY 10000

BalanduinoBluetooth * blue;

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
volatile int count = 0;
volatile int speed = 0;
// ISR(TIMER1_COMPA_vect){

// 		count = 0;
// 		digitalWrite(5, HIGH);
// 		// delayMicroseconds(1);
// 		digitalWrite(5, LOW);
// }
volatile uint16_t counter_m[2];        // counters for periods
volatile uint16_t period_m[2][8];      // Eight subperiods 
volatile uint8_t period_m_index[2];    // index for subperiods

int16_t speed_m[2];           // Actual speed of motors
uint8_t dir_m[2];             // Actual direction of steppers motors

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
  
  // DEBUG
  /*
  if ((motor==0)&&((debug_counter%10)==0)){
    Serial.print(1000.0/absSpeed);Serial.print("\t");Serial.print(absSpeed);Serial.print("\t");
    Serial.print(period_m[motor][0]);Serial.print("-");
    Serial.print(period_m[motor][1]);Serial.print("-");
    Serial.print(period_m[motor][2]);Serial.print("-");
    Serial.println(period_m[motor][3]);
    }
  */  
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
float blueSpeed = 0;

void loop(){
  setMotorSpeed(LEFT_MOTOR,200);
  setMotorSpeed(RIGHT_MOTOR,200);

}

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
  digitalWrite(MOTOR_ENABLE,LOW);
}


void setup() { 



	timer1_init();
	
  blue = new BalanduinoBluetooth();
  Serial.begin(9600);
  motorInit();
  setMotorSpeed(1,10000);

}