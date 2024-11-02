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

#define standbypin 8

// For PID Controller 
float Kp = 24;             // (P)roportional Tuning Parameter
float Ki = 3;             // (I)ntegral Tuning Parameter        
float Kd = 15;             // (D)erivative Tuning Parameter
float targetAngle = -1.5;  // Can be adjusted according to centre of gravity 
float time, currentTime, previousTime;

float lastpitch = 0;          // Keeps track of error over time
float iTerm;              // Used to accumulate error (integral)

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
  mpu6050.calcGyroOffsets(true);
}

void loop() {
  digitalWrite(standbypin, HIGH);
  mpu6050.update();
  Serial.println(mpu6050.getAngleX());
  Serial.println(PID());
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
  // Calculate pitch
  float pitch = mpu6050.getAngleX();
  
  // Calculate Error
  float error = targetAngle - pitch;
  if (abs(error) <= 1.5) error = 0;

  // Calculate PID terms
  float pTerm = Kp * error;
  time = currentTime - previousTime;
  iTerm += Ki * error * time;
  float dTerm = Kd * (pitch - lastpitch) / time;
  lastpitch = pitch;

  // Obtain PID output value
  float PIDValue = pTerm + iTerm + dTerm;

  // Cap values so be able to send the correct PWM signal to the motors
  if (PIDValue > 255) PIDValue = 255;
  else if (PIDValue < -255) PIDValue = -255;
  previousTime = millis();
  return int(PIDValue);
}
