#include <SoftPWM.h>
#include <KK2LCD.h>
#include <Motor.h>
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
const int switch_release_debounce_us = 100; 
const int switch_press_debounce_uS = 500; 


// Motor leftMotor(in,A5,A4);
// Motor rightMotor(11,A2,A3);
Motor frontMotor(IN2,IN1,IN3);
Motor leftMotor(OUT7,OUT8,OUT6);
Motor rightMotor(OUT3,OUT4,OUT5);
void setup() {
	SoftPWMBegin();
	// digitalWrite(IN3, HIGH);
}

void loop() {
	// // Serial.println("forward");
	// leftMotor.forward(100);
	// frontMotor.forward(100);
	// rightMotor.forward(100);
	// delay(500);
	// // frontMotor.stop();
	// // // // Serial.println("backward");
	// leftMotor.backward(255);
	// frontMotor.backward(255);
	// rightMotor.backward(255);
	// delay(2000);
	// frontMotor.stop();
	// leftMotor.stop();
	// rightMotor.stop();
	// delay(1000);
}
