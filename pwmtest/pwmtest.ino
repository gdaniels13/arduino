#include <SoftPWM.h>
#include <KK2LCD.h>
#include <Motor.h>
#include <EEPROM.h>
int WLED = 5;
const byte IN1 = 0;  //PD3 (PCINT27/TXD1/INT1)  not tested, but use Serial1 
const byte IN2 = 1;  //PD2 (PCINT26/RXD1/INT0)  interrupts good for CCPM decoding.
const byte IN3 = 2;  //PD0 (PCINT24/RXD0/T3)  tx0 is on the lcd not sure if using this would conflict with the lcd  
const byte IN4 = 3;  //PB2 (PCINT10/INT2/AIN0)
const byte IN5 = 4;  //PB0 (PCINT8/XCK0/T0)  //timer/counter0 source
const byte OUT1 = 5;  //PC6 (TOSC1/PCINT22)   //32.768kHz crystal or custom clock source for counter (rpm sensor)
const byte OUT2 = 6;  //PC4 (TDO/PCINT20)   //JTAG 
const byte OUT3 = 7;  //PC2 (TCK/PCINT18)   //JTAG
const byte OUT4 = 8;  //PC3 (TMS/PCINT19)  //JTAG
const byte OUT5 = 9;  //PC1 (SDA/PCINT17)  //I2C      i2c not tested
const byte OUT6 = 10; //PC0 (SCL/PCINT16)  //I2C
const byte OUT7 = 11; //PC5 (TDI/PCINT21)   //JTAG
const byte OUT8 = 12; //PC7 (TOSC2/PCINT23)   //32.768kHz crystal
const byte RED_LED = 13;  //PB3 (PCINT11/OC0A/AIN1)  //same as arduino!
const byte BUT1 = 14;  //PB7 (PCINT15/OC3B/SCK)    PWM     pwm not tested
const byte BUT2 = 15;  //PB6 (PCINT14/OC3A/MISO)   PWM
const byte BUT3 = 16;  //PB5 (PCINT13/ICP3/MOSI)
const byte BUT4 = 17;  //PB4 (PCINT12/OC0B/SS)
const byte _BUZZER = 18;  //PB1 (PCINT9/CLKO/T1)   CLOCK output can adjust with system prescaler. (make tones) not tested
const byte BATT = 3;
const int switch_release_debounce_us = 100; 
const int switch_press_debounce_uS = 500; 

String Str;
char str[7];
byte position = EEPROM.read(0);


Motor frontMotor(IN2,IN1  ,IN3);
Motor leftMotor(OUT7,OUT8,OUT6);
Motor rightMotor(OUT3,OUT4,OUT5);

void setup()
{
  st7565ClearBuffer();
  st7565Init( Font5x7 );
  st7565SetBrightness(12);
  st7565Refresh();
  enableButtons();
  SoftPWMBegin();
  analogReference(EXTERNAL); //important!!
  updateLCD();
  start();
  // countDown(15);
}

void enableButtons(){
  pinMode(BUT1,INPUT);
  digitalWrite(BUT1, HIGH);  
  pinMode(BUT2,INPUT);
  digitalWrite(BUT2, HIGH);
  pinMode(BUT3,INPUT);
  digitalWrite(BUT3, HIGH);
  pinMode(BUT4,INPUT);
  digitalWrite(BUT4, HIGH);
  pinMode(RED_LED, OUTPUT);
}

boolean buttonPressed(byte button)
{
  if(!digitalRead(button))
  {
    delayMicroseconds(switch_press_debounce_uS);
    if(!digitalRead(button))
    {
      while(!digitalRead(button))
      {
        digitalWrite(RED_LED,HIGH);
        //we could put a beep in here too.
      }
      delayMicroseconds(switch_release_debounce_us);
      digitalWrite(RED_LED,LOW);    
      return true;
    }
  }
  return false;
}


void loop() 
{
  // st7565ClearBuffer();
  int buttonPressed = getInput();
  // drawNum(buttonPressed,0,0);
  // delay(1000);
  // st7565Refresh();
  switch(buttonPressed){
    case BUT1:
      moveLeft();
      updateLCD();
      break;
    case BUT2:
      moveRight();
      updateLCD();
      break;
    case BUT3: 
      start();
      break;
    case BUT4:
      stop();
      updateLCD();
      break;
    case 5:
      preRace();
      break;
  }
}

void preRace(){
  stop();
  st7565ClearBuffer();
  st7565DrawString_P(0,0,PSTR("Press Button 2"));
  st7565Refresh();
  while(getInput()!=BUT2);
  countDown(15);
  start();
  updateLCD();
}

void updateLCD(){
  st7565SetFont(Font5x7);
  st7565ClearBuffer();
  
  //draw batt level
  float aread = analogRead(BATT)*.02618390805;
  
  Str = String(aread);    
  Str.toCharArray(str,6);  
  st7565DrawString_P(0,48,PSTR("BATT :"));
  st7565DrawString(6*6,48,str);


  //draw slow wheel speed

  Str = String(position);

  Str.toCharArray(str,6);  
  st7565DrawString_P(0,36,PSTR("SPEED :"));
  st7565DrawString(6*8,36,str);

  drawRect(position,1,10,4);
  drawScale();
  st7565Refresh();
}

void start(){
  rightMotor.forward(255);
  leftMotor.forward(position);
  frontMotor.forward(255);
  // if(position==62){
  //   rightMotor.forward(255);
  //   leftMotor.forward(255);
  // }
  // else if (position < 62){
  //   int diff = 63 - position;
  //   leftMotor.forward(calculateSpeed());
  //   rightMotor.forward(255);
  //   frontMotor.forward(255);
  //   // digitalWrite(IN2, HIGH);

  // }
  // else if (position>62){
  //   int diff =  position - 63;
  //   leftMotor.forward(255);
  //   rightMotor.forward(calculateSpeed());
  //   frontMotor.backward(255);
  //   // digitalWrite(IN2, HIGH);
  // }
}

void drawNum(int num,int x, int y){
  Str=String(num);
  Str.toCharArray(str,6);  
  st7565DrawString(x,y,str);
}

void countDown(int val){
  st7565SetFont( Font12x24Numbers );
  for(int i=val; i>=0; i--){
      st7565ClearBuffer();
      Str=String(i);
      Str.toCharArray(str,6);  
      st7565DrawString(52,26,str);
      st7565Refresh();
      delay(900);
  }
}

int calculateSpeed(){
  int diff;
  if(position > 62){
    diff =  position - 63;
  }
  else {
    diff = 63 - position;
  }
  return 255- (diff*4);
}

void stop(){
  frontMotor.stop();
  leftMotor.stop();
  rightMotor.stop();
}


void drawScale(){
    for(int i=-1; i<128; i+=16){
      drawRect(i,0,12,2);
    }
    drawRect(63,0,15,2);
}

void moveLeft(){
  position-=1;
  EEPROM.write(0,position);
}

void moveRight(){
  position+=1;
  EEPROM.write(0,position);
}

void drawRect(int x, int y,int height, int width ){
  for(int i=0; i<width; i++){
    for(int j=0; j<height; j++){
      st7565DrawPixel(x+i,y+j );
    }
  }
}

void clearRect(int x, int y,int height, int width ){
  for(int i=0; i<width; i++){
    for(int j=0; j<height; j++){
      st7565ClearPixel(x+i,y+j );
    }
  }
}

byte getInput(){
  boolean b1 = buttonPressed(BUT1);
  boolean b2 = buttonPressed(BUT2);
  boolean b3 = buttonPressed(BUT3);
  boolean b4 = buttonPressed(BUT4);

  if(b4&&b1){
    return 5;
  }
  if(b1){
    return BUT1;
  }
  else if(b2){
    return BUT2;
  }
  else if(b3){
    return BUT3;
  }
  else if(b4){
    return BUT4;
  }
  return -1;
}
