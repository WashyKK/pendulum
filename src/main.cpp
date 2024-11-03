#include <Arduino.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);

// Motor Control
#define enA 9
#define in1 6
#define in2 7

#define enB 10
#define in3 12
#define in4 13

#define m1a 4
#define m2a 2

#define standbypin 8

// For PID Controller 
float setPoint = -1.6;
float error, angleZ, currentTime, elapsedTime, previousTime;
float lastError, rateError, cumError;
float output = 0;
float Kp = 26.0 ;
float Kd = 0 ;
float Ki = 0;

float lastpitch = 0;      
float iTerm;              

void MotorDriver(int PIDValue);
int PID();

void setup() {
  // Pin definitions
  pinMode(standbypin, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(false,0,0);
}

void loop() {
  digitalWrite(standbypin, HIGH);
  mpu6050.update();
  // Serial.print("Angle X");
  // Serial.println(mpu6050.getAngleX());
  // Serial.println(PID());
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
  currentTime = millis();
  elapsedTime = (double)(currentTime - previousTime)/1000;

  // Calculate pitch
  float pitch = mpu6050.getAngleX();
  
  // Calculate Error
  error = setPoint - pitch;  
  // Serial.print("Error: ");
  // Serial.println(error);
  if (abs(error) <= 0.25) error = 0;
  cumError = cumError + error*elapsedTime;
  rateError = (error - lastError)/elapsedTime;

  // Serial.print("KP - KI - KD:  ");
  // Serial.print(Kp*error);
  // Serial.print("--");
  // Serial.print(Ki*cumError);
  // Serial.print("--");
  // Serial.println(Kd*rateError);
  // Obtain PID output value
  output = 0 + Kp*error + Ki*cumError + Kd*rateError;

  lastError = error;
  previousTime = currentTime;

  // Cap values so be able to send the correct PWM signal to the motors
  if (output > 255) output = 255;
  else if (output < -255) output = -255;

  previousTime = millis();
  return int(output);
}
