#include <Arduino.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);

// Motor Control
#define enA 3
#define in1 4
#define in2 5

#define enB 12
#define in3 10
#define in4 11

// For PID Controller 
float Kp = 1;             // (P)roportional Tuning Parameter
float Ki = 2;             // (I)ntegral Tuning Parameter        
float Kd = 3;             // (D)erivative Tuning Parameter
float targetAngle = 0;  // Can be adjusted according to centre of gravity 

float lastpitch = 0;          // Keeps track of error over time
float iTerm;              // Used to accumulate error (integral)

void MotorDriver(int PIDValue);
int PID();

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
}

void loop() {
  mpu6050.update();

  MotorDriver(PID());
}

void MotorDriver(int PIDValue){
  if(PIDValue >= 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, PIDValue);
    
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, PIDValue);
    }
  else{
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, PIDValue * -1);
    
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enB, PIDValue * -1);
    }
}

int PID(){
  // Calculate pitch
  float pitch = mpu6050.getAngleX();
  
  // Calculate Error
  float error = targetAngle - pitch;

  // Calculate PID terms
  float pTerm = Kp * error;
  iTerm += Ki * error * 10;
  float dTerm = Kd * (pitch - lastpitch) / 10;
  lastpitch = pitch;

  // Obtain PID output value
  float PIDValue = pTerm + iTerm - dTerm;

  // Cap values so be able to send the correct PWM signal to the motors
  if (PIDValue > 255) PIDValue = 255;
  else if (PIDValue < -255) PIDValue = -255;
  
  return int(PIDValue);
}
