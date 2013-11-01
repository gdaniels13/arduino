#include <Bluetooth.h>
#include <IMU.h>
#include <Kalman.h>
#include <Wire.h>

IMU *Imu;
 
void setup() 
{



  //setup IMU12212
  Imu = new IMU();
  Serial.begin(57600);
  
  /* Setup motor pins to output */
  sbi(leftPwmPortDirection,leftPWM);
  sbi(leftPortDirection,leftA);
  sbi(leftPortDirection,leftB);
  sbi(rightPwmPortDirection,rightPWM);
  sbi(rightPortDirection,rightA);
  sbi(rightPortDirection,rightB);  

  /* Set PWM frequency to 20kHz - see the datasheet http://www.atmel.com/Images/doc8025.pdf page 128-135 */
  // Set up PWM, Phase and Frequency Correct on pin 9 (OC1A) & pin 10 (OC1B) with ICR1 as TOP using Timer1
  TCCR1B = _BV(WGM13) | _BV(CS10); // Set PWM Phase and Frequency Correct with ICR1 as TOP and no prescaling
  ICR1H = (PWMVALUE >> 8); // ICR1 is the TOP value - this is set so the frequency is equal to 20kHz
  ICR1L = (PWMVALUE & 0xFF);

  /* Enable PWM on pin 9 (OC1A) & pin 10 (OC1B) */
  // Clear OC1A/OC1B on compare match when up-counting
  // Set OC1A/OC1B on compare match when downcountin
  TCCR1A = _BV(COM1A1) | _BV(COM1B1);
  setPWM(leftPWM,0); // Turn off pwm on both pins
  setPWM(rightPWM,0);

/* Calibrate the gyro and accelerometer relative to ground */
//  calibrateSensors();
  readFromSerial = false;
  /* Setup timing */
  loopStartTime = micros();
  timer = loopStartTime;
  gyroAngle = 0;
}

void loop() {
//  /* Calculate pitch */
//  accYangle = getAccY();
//  gyroYrate = getGyroYrate();
  Imu->getPitchAndRate(accYangle,gyroYrate);
  

  gyroAngle += gyroYrate*((float)(micros()-timer)/1000000);
  // See my guide for more info about calculation the angles and the Kalman filter: http://arduino.cc/forum/index.php/topic,58048.0.htm
//  pitch = kalman.getAngle(accYangle, gyroYrate, (float)(micros() - timer)/1000000); // Calculate the angle using a Kalman filter
  timer = micros();  

  /* Drive motors */
  // If the robot is laying down, it has to be put in a vertical position before it starts balancing
  // If it's already balancing it has to be ±45 degrees before it stops trying to balance
  if((layingDown && (pitch < -10 || pitch > 10)) || (!layingDown && (pitch < -45 || pitch > 45))) {
    layingDown = true; // The robot is in a unsolvable position, so turn off both motors and wait until it's vertical again
    stopAndReset();
  } 
  else {
    layingDown = false; // It's no longer laying down
    PID(targetAngle,targetOffset,turningOffset);        
  }



  /* Use a time fixed loop */
  lastLoopUsefulTime = micros() - loopStartTime;
  if (lastLoopUsefulTime < STD_LOOP_TIME) {
    while((micros() - loopStartTime) < STD_LOOP_TIME);
  }
  loopStartTime = micros();    
}

char* doubleToString(float number, int length)
{
  char* toReturn = new char[2];
  int t = number;
  
  toReturn[0] = t / 10;  
  toReturn[1] = t%10;
  
  return toReturn;
  
}


void PID(float restAngle, float offset, float turning) {
  /* Steer robot */

  /* Brake */
    if (restAngle < -20) // Limit rest Angle
      restAngle = -20;
    else if (restAngle > 20)
      restAngle = 20;
  
  
  /* Update PID values */
  float error = (restAngle - pitch);
  float pTerm = Kp * error;
  iTerm += Ki * error;
  float dTerm = Kd * (error - lastError);
  lastError = error;
  float PIDValue = pTerm + iTerm + dTerm;

  /* Steer robot sideways */
  float PIDLeft;
  float PIDRight;
  if (steerLeft) {
    if(turning < 0)
      turning = 0;
    PIDLeft = PIDValue-turning;
    PIDRight = PIDValue+turning;
  }
  else if (steerRight) {
    if(turning < 0)
      turning = 0;
    PIDLeft = PIDValue+turning;
    PIDRight = PIDValue-turning;
  }
  else {
    PIDLeft = PIDValue;
    PIDRight = PIDValue;
  }

  //PIDLeft *= 0.95; // compensate for difference in the motors

  /* Set PWM Values */
  if (PIDLeft >= 0)
    moveMotor(left, forward, PIDLeft);
  else
    moveMotor(left, backward, PIDLeft * -1);
  if (PIDRight >= 0)
    moveMotor(right, forward, PIDRight);
  else
    moveMotor(right, backward, PIDRight * -1);
}

void readSPP() {
  if(Serial.available()) {
      char input[30];
      uint8_t i = 0;
      while (1) {
        input[i] = Serial.read();
        if(input[i] == -1) // Error while reading the string
          return;
        if (input[i] == ';') // Keep reading until it reads a semicolon
          break;
        i++;
      }      
      /*Serial.print("Data: ");
      Serial.write((uint8_t*)input,i);
      Serial.println();*/
      if(input[0] == 'A') { // Abort
        stopAndReset();
        while(Serial.read() != 'C'); // Wait until continue is send
      } 
      
      /* Set PID and target angle */
      else if(input[0] == 'P') {
        strtok(input, ","); // Ignore 'P'
        Kp = atof(strtok(NULL, ";"));
      } else if(input[0] == 'I') {
        strtok(input, ","); // Ignore 'I'
        Ki = atof(strtok(NULL, ";"));  
      } else if(input[0] == 'D') {
        strtok(input, ","); // Ignore 'D'
        Kd = atof(strtok(NULL, ";"));  
      } else if(input[0] == 'T') { // Target Angle
        strtok(input, ","); // Ignore 'T'
        targetAngle = atof(strtok(NULL, ";"));  
      } else if(input[0] == 'G') { // The processing/Android application sends when it need the current values
        if(input[1] == 'P') // PID Values
          sendPIDValues = true;
        else if(input[1] == 'B') // Begin
          sendData = true; // Send output to processing/Android application
        else if(input[1] == 'S') // Stop
          sendData = false; // Stop sending output to processing/Android application
      }
      /* Remote control */
      else if(input[0] == 'S') // Stop
        steer(stop);      
      else if(input[0] == 'J') { // Joystick
        strtok(input, ","); // Ignore 'J'
        sppData1 = atof(strtok(NULL, ",")); // x-axis
        sppData2 = atof(strtok(NULL, ";")); // y-axis
        steer(joystick);
      }
      else if(input[0] == 'M') { // IMU
        strtok(input, ","); // Ignore 'I'
        sppData1 = atof(strtok(NULL, ",")); // Pitch
        sppData2 = atof(strtok(NULL, ";")); // Roll
        steer(imu);
        //SerialBT.printNumberln(sppData1);
        //SerialBT.printNumberln(sppData2);
      }
    
  } 
  else
  {
    steer(stop);
  }
}




//
//void calibrateSensors() {
//  for (uint8_t i = 0; i < 100; i++) { // Take the average of 100 readings
//    zeroValues[0] += analogRead(gyroY);
//    zeroValues[1] += analogRead(accY);
//    zeroValues[2] += analogRead(accZ);
//    delay(10);
//  }
//  zeroValues[0] /= 100; // Gyro X-axis
//  zeroValues[1] /= 100; // Accelerometer Y-axis
//  zeroValues[2] /= 100; // Accelerometer Z-axis
//
//  if(zeroValues[1] > 500) { // Check which side is lying down - 1g is equal to 0.33V or 102.3 quids (0.33/3.3*1023=102.3)
//    zeroValues[1] -= 102.3; // +1g when lying at one of the sides
//    //kalman.setAngle(90); // It starts at 90 degress and 270 when facing the other way
//    gyroAngle = -90;
//  } else {
//    zeroValues[1] += 102.3; // -1g when lying at the other side
//    //kalman.setAngle(270);
//    gyroAngle = 90;
//  }
//
//  digitalWrite(buzzer,HIGH);
//  delay(100);  
//  digitalWrite(buzzer,LOW);
//}
void moveMotor(Command motor, Command direction, float speedRaw) { // Speed is a value in percentage 0-100%
  if(speedRaw > 100)
    speedRaw = 100;
  int speed = speedRaw*((float)PWMVALUE)/100; // Scale from 100 to PWMVALUE
  if (motor == left) {
    setPWM(leftPWM,speed); // Left motor pwm
    if (direction == forward) {
      cbi(leftPort,leftA);
      sbi(leftPort,leftB);
    } 
    else if (direction == backward) {
      sbi(leftPort,leftA);
      cbi(leftPort,leftB);
    }
  } 
  else if (motor == right) {
    setPWM(rightPWM,speed); // Right motor pwm
    if (direction == forward) {
      cbi(rightPort,rightA);
      sbi(rightPort,rightB);
    } 
    else if (direction == backward) {
      sbi(rightPort,rightA);
      cbi(rightPort,rightB);
    }
  }
}




