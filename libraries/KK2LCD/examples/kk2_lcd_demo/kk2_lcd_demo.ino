//Arduino on the KK2 test code by Marc Griffith is licensed under a Creative Commons Attribution 3.0 Unported License.
//http://creativecommons.org/licenses/by/3.0/deed.en_US

//instructions and videos on getting this working with Arduino are available here
//http://www.rchacker.com/diy/arduino-on-the-kk2-multicopter-controller


#include <KK2LCD.h>

//AIL, THR etc
//can also be digital outputs

const byte IN1 = 0;  //PD3 (PCINT27/TXD1/INT1)  not tested, but use Serial1 
const byte IN2 = 1;  //PD2 (PCINT26/RXD1/INT0)  interrupts good for CCPM decoding.
const byte IN3 = 2;  //PD0 (PCINT24/RXD0/T3)  tx0 is on the lcd not sure if using this would conflict with the lcd  
const byte IN4 = 3;  //PB2 (PCINT10/INT2/AIN0)
const byte IN5 = 4;  //PB0 (PCINT8/XCK0/T0)  //timer/counter0 source

//motor outputs can also be digital inputs. these also have PCINT16 to 23 Arduino interrupts not tested.
const byte OUT1 = 5;  //PC6 (TOSC1/PCINT22)   //32.768kHz crystal or custom clock source for counter (rpm sensor)
const byte OUT2 = 6;  //PC4 (TDO/PCINT20)   //JTAG 
const byte OUT3 = 7;  //PC2 (TCK/PCINT18)   //JTAG
const byte OUT4 = 8;  //PC3 (TMS/PCINT19)  //JTAG
const byte OUT5 = 9;  //PC1 (SDA/PCINT17)  //I2C      i2c not tested
const byte OUT6 = 10; //PC0 (SCL/PCINT16)  //I2C
const byte OUT7 = 11; //PC5 (TDI/PCINT21)   //JTAG
const byte OUT8 = 12; //PC7 (TOSC2/PCINT23)   //32.768kHz crystal

const byte RED_LED = 13;  //PB3 (PCINT11/OC0A/AIN1)  //same as arduino!

//important enable the internal pullups when using these as inputs
const byte BUT1 = 14;  //PB7 (PCINT15/OC3B/SCK)    PWM     pwm not tested
const byte BUT2 = 15;  //PB6 (PCINT14/OC3A/MISO)   PWM
const byte BUT3 = 16;  //PB5 (PCINT13/ICP3/MOSI)
const byte BUT4 = 17;  //PB4 (PCINT12/OC0B/SS)

const byte _BUZZER = 18;  //PB1 (PCINT9/CLKO/T1)   CLOCK output can adjust with system prescaler. (make tones) not tested

//uncomment if you want to write your own LCD library
/*
const byte LCD_CS1 = 19;
const byte LCD_RES = 20;
const byte LCD_A0 = 21;
const byte LCD_SCL = 22;
const byte LCD_SI = 23;
*/

//analog reads must be done using thier channels, specifying digital pin numbers will not work in this case
const byte BATT = 3;

const byte GYR_R = 1;
const byte GYR_Y = 2;
const byte GYR_P = 4;

const byte ACC_X = 5;
const byte ACC_Y = 6;
const byte ACC_Z = 7;

//most of the hardware pwm is on the LCD, LED pins so dont bother. There is PWM on the buttons.

const int switch_release_debounce_us = 100; //milliseconds
const int switch_press_debounce_uS = 500; //microseconds

void setup() {
  // put your setup code here, to run once:

  pinMode(RED_LED, OUTPUT); 

  pinMode(GYR_R, INPUT);
  pinMode(GYR_Y, INPUT);
  pinMode(GYR_P, INPUT);

  pinMode(ACC_X, INPUT);
  pinMode(ACC_Y, INPUT);
  pinMode(ACC_Z, INPUT);

  pinMode(BUT1,INPUT);
  digitalWrite(BUT1, HIGH);   //enable internal pullup.

  pinMode(BUT2,INPUT);
  digitalWrite(BUT2, HIGH);

  pinMode(BUT3,INPUT);
  digitalWrite(BUT3, HIGH);

  pinMode(BUT4,INPUT);
  digitalWrite(BUT4, HIGH);

  analogReference(EXTERNAL); //important!!

  st7565Init( Font5x7 );
  st7565SetBrightness(12);
  st7565DrawString_P( 64, 40, PSTR("Arduino on") );
  st7565DrawString_P( 64, 32, PSTR(" the KK2.") );  
  st7565DrawString_P( 64, 24, PSTR("Test suite") );  
  st7565DrawString_P( 64, 16, PSTR("   v1.0  ") );
  st7565DrawString_P( 64, 8, PSTR( "  Marc G") );
  st7565DrawString_P( 50, 1, PSTR("Press any key") );
  st7565WriteLogo(); //see library to modify
  
  
  delay(1000);
  while(true)
  {
    if(!digitalRead(BUT1)||!digitalRead(BUT2)||!digitalRead(BUT3)||!digitalRead(BUT4)) {
      break;
    }
  }
}

String Str = String("hello hello");
char str[7];
int ii = 0;

// every time I write debounce code it comes out different. Complicated but I like the feel of it.
byte button4Pressed()
{
  if(!digitalRead(BUT4))
  {
    delayMicroseconds(switch_press_debounce_uS);
    if(!digitalRead(BUT4))
    {
      while(!digitalRead(BUT4))
      {
        st7565SetBrightness(12);
        st7565ClearBuffer();
        st7565SetFont( Font12x16 );
        st7565DrawString_P( 42, 26 ,  PSTR("Next Hello") );
        st7565Refresh();
        digitalWrite(RED_LED,HIGH);
        //we could put a beep in here too.
      }
      delayMicroseconds(switch_release_debounce_us);
      digitalWrite(RED_LED,LOW);    
      return 1;
    }
  }
  return 0;
}

void fonts()
{
  //delay(200);
//  st7565SetBrightness(12);
  while(true)
  {
    st7565ClearBuffer();
    st7565SetBrightness(12);
    st7565SetFont( Font12x16 );
    st7565DrawString_P( 0, 0,  PSTR("Font12x16") );
    st7565SetFont( Font12x24Numbers );
    st7565DrawString_P( 0+40, 15+12, PSTR("12") );
    st7565DrawString_P( 34+40, 15+12, PSTR("24") );
    st7565SetFont( Font12x16 );
    st7565DrawChar( 24+40, 20+12, (uint8_t) '.' );

    st7565SetFont( Font5x7 );
    st7565DrawString_P( 20, 14, PSTR("Font5x7") );
    st7565DrawString_P( 102, 54, PSTR("Next") );
    st7565Refresh(); 
    if(button4Pressed())    
      return;    
  }
}

/*
// needs fixing, causes memory leaks !!
 void drawCoordinates(int x, int y)
 {
 Str = String(x) + "," + String(y);
 Str.toCharArray(str,6);  
 st7565DrawString( x, y, str);     
 }
 
 void text_position()
 {
 delay(200);
 st7565SetBrightness(12);
 while(true)
 {
 st7565ClearBuffer();
 st7565SetBrightness(12);
 st7565SetFont( Font5x7 );
 st7565DrawString_P( 0, 8, PSTR("Co-ords test") );
 drawCoordinates(0,0);
 drawCoordinates(80,0);
 drawCoordinates(81,8);
 drawCoordinates(82,16);
 drawCoordinates(83,24);
 drawCoordinates(84,32);
 drawCoordinates(85,40);
 drawCoordinates(86,48);
 drawCoordinates(0,20);
 drawCoordinates(6,28);
 drawCoordinates(0,56);
 drawCoordinates(38,56);
 st7565DrawString_P( 102, 56, PSTR("Next") );
 st7565Refresh(); 
 if(button4Pressed())    
 return;    
 }
 }*/


void analog()
{
  int aread = 0;
  //  delay(switch_release_debounce);
  st7565SetBrightness(12);
  while(true)
  {
    st7565ClearBuffer();
    st7565SetBrightness(12);
    st7565SetFont( Font12x16 );
    st7565DrawString_P( 0, 0, PSTR("Analog Read") );
    st7565SetFont( Font5x7 );

    delayMicroseconds(10);
    aread = analogRead(GYR_R);
    Str = String(aread);    
    Str.toCharArray(str,6);  
    st7565DrawString_P(10*6,16,PSTR("GYR_R "));
    st7565DrawString(16*6,16,str);
    
    delayMicroseconds(10);
    aread = analogRead(GYR_Y);
    Str = String(aread);    
    Str.toCharArray(str,6);  
    st7565DrawString_P(10*6,24,PSTR("GYR_Y "));
    st7565DrawString(16*6,24,str);
    
    delayMicroseconds(10);
    aread = analogRead(GYR_P);
    Str = String(aread);    
    Str.toCharArray(str,6);  
    st7565DrawString_P(10*6,32,PSTR("GYR_P "));
    st7565DrawString(16*6,32,str);
    
    delayMicroseconds(10);
    aread = analogRead(ACC_X);
    Str = String(aread);    
    Str.toCharArray(str,6);  
    st7565DrawString_P(0,16,PSTR("ACC_X "));
    st7565DrawString(6*6,16,str);
    
    delayMicroseconds(10);
    aread = analogRead(ACC_Y);
    Str = String(aread);    
    Str.toCharArray(str,6);  
    st7565DrawString_P(0,24,PSTR("ACC_Y "));
    st7565DrawString(6*6,24,str);
    
    delayMicroseconds(10);
    aread = analogRead(ACC_Z);
    Str = String(aread);    
    Str.toCharArray(str,6);  
    st7565DrawString_P(0,32,PSTR("ACC_Z "));
    st7565DrawString(6*6,32,str);
    
    delayMicroseconds(10);
    aread = analogRead(BATT);
    Str = String(aread);    
    Str.toCharArray(str,6);  
    st7565DrawString_P(0,48,PSTR("BATT  "));
    st7565DrawString(6*6,48,str);
    
    st7565DrawString_P( 102, 56, PSTR("Next") );
    st7565Refresh(); 
    if(button4Pressed())    
      return;    
  }
}

void buttons()
{
  st7565SetBrightness(12);
  //  delay(switch_release_debounce);
  while(true)
  {
    st7565ClearBuffer();
    st7565SetFont( Font12x16 );
    st7565DrawString_P( 0, 0,  PSTR("  Buttons") );
    st7565SetFont( Font12x24Numbers );

    Str = String(digitalRead(BUT1));
    Str.toCharArray(str,5);  
    st7565DrawString( 0, 38, str);   

    Str = String(digitalRead(BUT2));
    Str.toCharArray(str,5);  
    st7565DrawString( 38, 38, str);   
    
    Str = String(digitalRead(BUT3));
    Str.toCharArray(str,5);  
    st7565DrawString( 76, 38, str);   

    st7565SetFont( Font5x7 );    
    st7565DrawString_P( 102, 56, PSTR("Next") );

    st7565Refresh();
    if(button4Pressed())    
      return;    
  }  
}

void leds()
{
  //  delay(switch_release_debounce);
  st7565SetBrightness(12);
  while(true)
  {
    byte offset = 0;
    for (int i = 0;i<14 ;i++)
    {
      st7565ClearBuffer();
      st7565SetFont( Font12x16 );
      st7565DrawString_P( 0, 0,  PSTR("Digital out") );


      Str = String(i);
      Str.toCharArray(str,5);  

      st7565SetFont( Font5x7 );
      st7565DrawString_P(0,28,PSTR("digitalWrite("));
      st7565DrawString(13*6,28,str);
      if(i>9)
      {
        offset = 6; 
      }
      st7565DrawString_P(14*6+offset,28,PSTR(",HIGH)"));
      st7565DrawString_P(20*6-2+offset,28,PSTR(";"));
      st7565DrawString_P( 102, 56, PSTR("Next") );

      /*
    st7565SetFont( Font12x24Numbers );
       st7565DrawString( 0+OFFSET_X, 15+OFFSET_Y, str);   
       */
      pinMode(i,OUTPUT);
      digitalWrite(i,HIGH);
      st7565Refresh();

      for(int k = 0;k<100;k++)
      {
        if(button4Pressed())    
          return;
        delay(5);
      }
      digitalWrite(i,LOW);
      pinMode(i,INPUT);

    }
  }

}


void contrast()
{
  while(true)
  {

    for (int i = 0;i<35;i++)
    { 
      st7565ClearBuffer();
      st7565SetFont( Font12x16 );
      st7565DrawString_P( 0, 0,  PSTR("SetBrightness()") );

      Str = String(i);
      Str.toCharArray(str,5);  
      st7565SetFont( Font12x24Numbers );
      st7565DrawString( 50, 20, str);   
      st7565SetBrightness(i);
      st7565SetFont( Font5x7 );
      st7565DrawString_P( 102, 56, PSTR("Next") );
      st7565Refresh();

      //gives us a delay of 250ms while still detecting button presses
      for(int k = 0;k<50;k++)
      {
        if(button4Pressed())    
          return;
        delay(5);
      }
    }
  }
}


void time()
{
  unsigned long time = 0;
  st7565SetBrightness(12);
  st7565ClearBuffer();
  st7565SetFont( Font12x16 );
  st7565DrawString_P( 0, 0,  PSTR("Test clock") );
  st7565SetFont( Font5x7 );
  st7565DrawString_P( 4, 24, PSTR("One flash per second.") );
  st7565DrawString_P( 102, 56, PSTR("Next") );
  st7565Refresh();

  while(true)
  {
    // heartbeat
    if (millis() % 1000)
    {
      digitalWrite(RED_LED,LOW);
    }
    else
    {
      digitalWrite(RED_LED,HIGH);
      delay(100);
    }
    if(button4Pressed())    
      return;
  }
}

void loop() {
  analog();
  buttons();
  leds();
  fonts();
  contrast();
  time();
  //text_position()
}


