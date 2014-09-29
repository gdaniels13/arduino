
//Create variables for 6 channels
int RXCH[2]; 
volatile int RXSG[2];
int RXOK[2];
int PWMSG[2];

void setup() {
  
  //Start communication to serial port
  Serial.begin(115200);  
  
  //Assign PPM input pins. The receiver output pins are conected as below to non-PWM Digital connectors:
  RXCH[0] = 11;  //Throttle
  RXCH[1] = 6;  //Aile / Yaw
  // RXCH[2] = 5;  //Elev. / Pitch
  // RXCH[3] = 2;  //Rudd. / Roll
  // RXCH[4] = 7;  //Gear
  // RXCH[5] = 8;  //Aux / Flt Mode
  
  for (int i = 0; i < 2; i++){
    pinMode(RXCH[i], INPUT);
  }
}

void loop() {
  
// Read RX values 
  for (int i = 0; i < 2; i++){                                        //for each of the 6 channels: 
  RXSG[i] = pulseIn(RXCH[i], HIGH, 20000);                            //read the receiver signal
  if (RXSG[i] == 0) {RXSG[i] = RXOK[i];} else {RXOK[i] = RXSG[i];}    //if the signal is good then use it, else use the previous signal
  PWMSG[i] = map(RXSG[i], 1000, 2000, 0, 511);                        //substitute the high values to a value between 0 and 511
  constrain (PWMSG[i], 0, 511);                                       //make sure that the value stays within the disired boundries
  
 // Print RX values
  Serial.print(" ||   Ch: ");
  Serial.print(i+1);
  Serial.print(" / PWMSG: ");
  Serial.print(PWMSG[i]);
  //Serial.print(" / RXSG: ");  
  //Serial.print(RXSG[i]);
  delay(10);  
 }
 Serial.println();
}