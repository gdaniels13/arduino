// Sweep
// by BARRAGAN <http://barraganstudio.com> 
// This example code is in the public domain.


#include <Servo.h> 
 
Servo myservo;
                // a maximum of eight servo objects can be created 
  int led = 13;
  int trigger = 5;
  int val = 0;
void setup() 
{ 
  pinMode(trigger, INPUT);
  pinMode(led, OUTPUT);    
//   blinkFast();   
  myservo.attach(2);  // attaches the servo on pin 9 to the servo object 
  myservo.write(55);
 
 
  delay(2000);
} 
 
 
void loop() 
{
  blinkFast(); //indicates ready;
    digitalWrite(13,LOW);
  arm();
  tri(650);
  
     myservo.write(170);
    
    digitalWrite(13,LOW);
    delay(1000);    
    
    myservo.write (40);
   
} 

void tri(int d){
  int val;
  while(true){
    val = analogRead(0);            // reads the value of the potentiometer (value between 0 and 1023) 
    if(val >d){
      return;
    }
  }
}

void arm(){
    digitalWrite(13,LOW);
  while(digitalRead(trigger)==LOW);
    digitalWrite(13,HIGH);
}

void blinkFast(){
  int d = 25;
  for(int i = 0; i<20; ++i)
  {
     digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(d);               // wait for a second
    digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
    delay(d); 
  }
}


