volatile int count = 0;

 void setup() { 
   Serial.begin (115200);
   attachInterrupt(0, interupt, CHANGE);
 } 

 void loop() { 
	Serial.println(count);
 } 


void interupt(){
	count++;
}