#include <Arduino.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

// Initialize MPU6050 object with Wire library
MPU6050 mpu6050(Wire);

// Motor control pins
#define enA 9   // Enable pin for motor A (PWM speed control)
#define in1 6   // Direction control pin for motor A
#define in2 7   // Direction control pin for motor A

#define enB 10  // Enable pin for motor B (PWM speed control)
#define in3 12  // Direction control pin for motor B
#define in4 13  // Direction control pin for motor B

// Motor A encoder pins (optional, for feedback control)
#define m1a 4
#define m2a 2

#define standbypin 8  // Standby control pin for the motor driver

#define margin 0.1  // Small tolerance margin for error

// PID Controller variables
float setPoint = -1.58; // Target pitch angle, specific to the robot's balance point
float error, angleZ, currentTime, elapsedTime, previousTime;
float lastError, rateError, cumError;
float output = 0;  // Output from PID calculation
float Kp = 26.0 ;  // Proportional gain, tuned for this robot
float Kd = 0.005;  // Derivative gain, for rate of error change
float Ki = 0.0001; // Integral gain, for cumulative error correction

float lastpitch = 0;  // Last pitch angle
float iTerm;          // Integral term in PID

// Function declarations
void MotorDriver(int PIDValue);  // Controls motor speed and direction
int PID();                       // Calculates the PID output

void setup() {
  // Motor and standby pin setup
  pinMode(standbypin, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Initialize serial communication and MPU6050 sensor
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(false,0,0); // Optional: calibrates gyro offsets
}

void loop() {
  digitalWrite(standbypin, HIGH);  // Enable motor driver
  mpu6050.update();                // Update MPU6050 sensor values
  MotorDriver(PID());              // Drive motors based on PID output
}

void MotorDriver(int PIDValue) {
  // If PIDValue is positive, move motors forward
  if (PIDValue >= 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, PIDValue);  // Set motor A speed
    
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, PIDValue);  // Set motor B speed
  } 
  else { // If PIDValue is negative, move motors backward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, -PIDValue); // Set motor A speed (absolute value)
    
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enB, -PIDValue); // Set motor B speed (absolute value)
  }
}

int PID() {
  currentTime = millis(); // Current time in milliseconds
  elapsedTime = (double)(currentTime - previousTime) / 1000; // Elapsed time in seconds

  float pitch = mpu6050.getAngleX(); // Get current pitch angle from sensor
  
  error = setPoint - pitch; // Calculate error between target and current angle
  if (abs(error) <= margin) error = 0; // Ignore small errors within the margin
  
  cumError += error * elapsedTime; // Accumulate error over time
  rateError = (error - lastError) / elapsedTime; // Calculate rate of error change

  // PID formula
  output = 0 + Kp * error + Ki * cumError + Kd * rateError;

  lastError = error;        // Store current error for next cycle
  previousTime = currentTime; // Update previous time

  // Limit output to max PWM values (0-255)
  if (output > 255) output = 255;
  else if (output < -255) output = -255;

  return int(output); // Return PID output as an integer
}
