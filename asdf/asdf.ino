#include "Encoder.h"
#include <TimerOne.h>
Encoder encoder(A0,150);

void isr(){
  encoder.timerIsr();
}

 void setup() {
    Timer1.initialize(5000);
    Timer1.attachInterrupt(isr);
   Serial.begin(115200); 
 }

 void loop() {
    Serial.println(encoder.read());


  }

