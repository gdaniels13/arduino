#include <cstdlib.h>
#include <Wire.h>
#include <IMU.h>
#include <Kalman.h>

IMU* imu;

float angle;
float accelAngle;
float gyro;
float targetAngle=15;
char stringBuf[30];
char tBuf[10];
void setup()
{
  Serial.begin(57600);
  imu = new IMU();
  
}


void loop()
{
  imu->getCurAngleKalman(angle,gyro,accelAngle);
  
  BTprint();
  delay(10);
}

void BTprint()
{
      Serial.print("V,");
      Serial.print(accelAngle+180);
      Serial.print(',');
      Serial.print(gyro+180);
      Serial.print(',');
      Serial.println(angle+180);
}


// void sendBluetoothData() {
//   if((millis() - dataTimer > 50)) 
//   {  // Only send data every 50ms
//     if(sendPairConfirmation) 
//     {
//       sendPairConfirmation = false;
//       dataTimer = millis(); // Reset the timer, to prevent it from sending data in the next loop
      
//       SerialBT.println("WC");
//     } 
//     else if(sendPIDValues) 
//     {
//       sendPIDValues = false;
//       dataTimer = millis(); // Reset the timer, to prevent it from sending data in the next loop
      
//       SerialBT.print("P,");
//       SerialBT.print(Kp);
//       SerialBT.print(',');
//       SerialBT.print(Ki);
//       SerialBT.print(',');
//       SerialBT.print(Kd);
//       SerialBT.print(',');
//       SerialBT.println(targetAngle);
//     } 
//     else if(sendSettings) 
//     {
//       sendSettings = false;
//       dataTimer = millis(); // Reset the timer, to prevent it from sending data in the next loop
      
//       SerialBT.print("S,");
//       SerialBT.print(BackToSpot);
//       SerialBT.print(',');
//       SerialBT.print(controlAngleLimit);
//       SerialBT.print(',');
//       SerialBT.println(turningAngleLimit);
//     } 
//     else if(sendInfo) 
//     {      
//       sendInfo = false;
//       dataTimer = millis(); // Reset the timer, to prevent it from sending data in the next loop
      
//       SerialBT.print("I,");
//       SerialBT.print(version);
      
//       #if defined(__AVR_ATmega644__)
//         SerialBT.print(",ATmega644,");
//       #elif defined(__AVR_ATmega1284P__)
//         SerialBT.print(",ATmega1284P,");
//       #else
//         SerialBT.print(",Unknown,");
//       #endif
      
//       SerialBT.print(batteryLevel);
//       SerialBT.print("V,");
//       SerialBT.println((double)millis()/60000.0);
//     } 
//     else if(sendData) 
//     {
//       dataTimer = millis();
      
//       SerialBT.print("V,");
//       SerialBT.print(accAngle);
//       SerialBT.print(',');
//       SerialBT.print(gyroAngle);
//       SerialBT.print(',');
//       SerialBT.println(pitch);
//     }
//   }
// }


