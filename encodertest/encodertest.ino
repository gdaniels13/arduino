#include <Encoder.h>

// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder encoder(2,3);

void setup() {
  Serial.begin(115200);
  Serial.println("TwoKnobs Encoder Test:");
  
}

long positionLeft  = -999;

void loop() {
  long newLeft;
  newLeft = encoder.read();
  if (newLeft != positionLeft) {
    Serial.print(newLeft);
    Serial.println();
    positionLeft = newLeft;
  }
  // if a character is sent from the serial monitor,
  // reset both back to zero.
  if (Serial.available()) {
    Serial.read();
    Serial.println("Reset both knobs to zero");
    encoder.write(0);
  }
}