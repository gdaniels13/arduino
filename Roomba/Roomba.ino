
 #include <SoftwareSerial.h>

int rxPin = 3;
int txPin = 4;
int ledPin = 13;

SoftwareSerial Roomba(rxPin,txPin);
char sensorbytes[100];

#define bumpright (sensorbytes[0] & 0x01)
#define bumpleft !(1 >> (sensorbytes[0] & 0x02))
#define leftCliff ((byte)sensorbytes[2])
#define leftFrontCliff ((byte)sensorbytes[3])
#define leftFrontCliff ((byte)sensorbytes[3])
#define rightFrontCliff ((byte)sensorbytes[4])
#define rightCliff ((byte)sensorbytes[5])

void setup() {
  pinMode(ledPin, OUTPUT);   // sets the pins as output
  Serial.begin(115200);
  Roomba.begin(115200);  
  digitalWrite(ledPin, HIGH); // say we're alive
  Serial.println ("Sending start command...");
  delay (1000);
   // set up ROI to receive commands  
  Roomba.write(128);  // START
  delay(150);
  Serial.println ("Sending Safe Mode command...");
  delay (1000);
  Roomba.write(132);  // CONTROL
  delay(150);
  digitalWrite(ledPin, LOW);  // say we've finished setup
  Serial.println ("Ready to go!");
  delay (500);
}

void loop() {
  // digitalWrite(ledPin, HIGH); // say we're starting loop
  // Serial.println ("Go Forward");
  // goForward();
  // delay (500);
  // Serial.println ("Halt!");
  // halt();
  // Serial.println ("Go Backwards");
  // delay (500);
  // goBackward();
  // delay (500);
  // Serial.println ("Halt!");
  // halt();
  // Serial.println("spinning");
  // delay(500);
  // spin();
  // delay(1000);
  // halt();
  // updateSensors();
  // printSensors();
  // delay(250);
  // Serial.print("Left Cliff");
  // get2ByteSensorPaket((byte)28);
    // Serial.println((unsigned short)get2ByteSensorPaket((byte)44));
    // bumpandgo();
    // Serial.println("going");
      // if (Roomba.available()) Serial.write(Roomba.read()); //mySerial is Tx from Roomba, Serial is USB to laptop
      if(Serial.available()) 
{
      Serial.write((byte)Serial.read());
      Serial.println("");
    }
}

void bumpandgo(){
  digitalWrite(ledPin, HIGH); // say we're starting loop
  updateSensors();
  digitalWrite(ledPin, LOW);  // say we're after updateSensors
  if(bumpleft) {
    spinRight();
    delay(1000);
  }
  else if(bumpright) {
    spinLeft();
    delay(1000);

  }
  goForward();

  }

void printSensors(){
	Serial.print("bumpLeft: ");
	Serial.print(bumpleft);
	Serial.print(" bumpRight: ");
	Serial.print(bumpright);
	Serial.print(" leftCliff: ");
	Serial.println(leftCliff);
}

void spinLeft() {
  Roomba.write(137);   // DRIVE
  Roomba.write((byte)0x00);   // 0x00c8 == 200
  Roomba.write((byte)0xc8);
  Roomba.write((byte)0x00);
  Roomba.write((byte)0x01);   // 0x0001 == spin left
}
void spinRight() {
  Roomba.write(137);   // DRIVE
  Roomba.write((byte)0x00);   // 0x00c8 == 200
  Roomba.write((byte)0xc8);
  Roomba.write((byte)0xff);
  Roomba.write((byte)0xff);   // 0xffff == -1 == spin right
}

void goForward() {
  Roomba.write(137);   // DRIVE
  Roomba.write((byte)0x00);   // 0x00c8 == 200
  Roomba.write(0xc8);
  Roomba.write(0x80);
  Roomba.write((byte)0x00);
}
void goBackward() {
  Roomba.write(137);   // DRIVE
  Roomba.write(0xff);   // 0xff38 == -200
  Roomba.write(0x38);
  Roomba.write(0x80);
  Roomba.write((byte)0x00);
}

void halt(){
 byte j = 0x00;
 Roomba.write(137);   
 Roomba.write(j);   
 Roomba.write(j);
 Roomba.write(j);
 Roomba.write(j);
}

unsigned int get2ByteSensorPaket(byte packetId){
  Roomba.write(149);
  Roomba.write(1);
  Roomba.write(packetId);
  delay(15);
  char buffer[2];
  Roomba.readBytes(buffer,2);
  Serial.print((byte)buffer[0],BIN);
  Serial.print("|");
  Serial.println((byte)buffer[1],BIN);
  unsigned int toReturn = buffer[0];

  toReturn = toReturn <<4;
  toReturn += buffer[1];
  return toReturn;
  // Serial.println(toReturn,BIN);

}


void updateSensors() {
  Roomba.write(142);
  Roomba.write((byte)100);  // sensor packet 1, 10 bytes
  delay(100); // wait for sensors 
  char i = 0;
  while(Roomba.available()) {
    int c = Roomba.read();
    if( c==-1 ) {
      for( int i=0; i<5; i ++ ) {   // say we had an error via the LED
        digitalWrite(ledPin, HIGH); 
        delay(50);
        digitalWrite(ledPin, LOW);  
        delay(50);
      }
    }
    sensorbytes[i++] = c;
  }    
}