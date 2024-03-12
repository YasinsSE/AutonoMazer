/**
 * @file AutonoMazer.ino
 * @brief Autonomous robot control program using ultrasonic sensors and Arduino Uno.
 * @author Yasin YILDIRIM
 * @date 03/2024
 * 
 * This program controls the movement of an autonomous robot using ultrasonic sensors
 * to measure distances to surrounding walls and navigate towards a goal position. It utilizes motor control
 * and sensor reading functions to make decisions based on sensor readings.
 */

#include <NewPing.h>

//---------------------------------------------------------------------
//---------------------Ultrasonic sensor pins--------------------------
//---------------------------------------------------------------------

#define frontTrig 2
#define frontEcho 3
#define leftTrig 4
#define leftEcho 5
#define rightTrig 6
#define rightEcho 7

#define MAX_DISTANCE 200

//---------------------------------------------------------------------
//---------------------------Motor pins--------------------------------
//---------------------------------------------------------------------

#define LEFT_MOTOR_ENABLE 8
#define LEFT_MOTOR_IN1 9
#define LEFT_MOTOR_IN2 10
#define RIGHT_MOTOR_ENABLE 11
#define RIGHT_MOTOR_IN1 12
#define RIGHT_MOTOR_IN2 13

//---------------------------------------------------------------------
//--------------Constants for motor speeds and timing------------------
//---------------------------------------------------------------------

NewPing sonarFront(frontTrig, frontEcho, MAX_DISTANCE);
NewPing sonarLeft(leftTrig, leftEcho, MAX_DISTANCE);
NewPing sonarRight(rightTrig, rightEcho, MAX_DISTANCE);

const int baseSpeed = 200; // Need optimizations
const int lExtraSpeedSpeed = 30;
const int turnTime = 350;
const int uTurnTime = 750;
const int goForwardTime = 250;
const int thershold = 10;
const int fThershold = 12;
/*
const int errorMargin = 5;
const int pathWidth = 20;
const int targetWidth = 40 */


int lDist, rDist, fDist;
int mean;
int sum;

int LmotorSpeed = baseSpeed, RmotorSpeed = baseSpeed;

//---------------------------------------------------------------------
//---Setup function to initialize serial communication and pin modes---
//---------------------------------------------------------------------

void setup() {
  Serial.begin(19200);
  pinMode(LEFT_MOTOR_ENABLE, OUTPUT);
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_ENABLE, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);

  pinMode(leftTrig, OUTPUT);  
	pinMode(leftEcho, INPUT);  
  pinMode(frontTrig, OUTPUT);  
	pinMode(frontEcho, INPUT);  
  pinMode(rightTrig, OUTPUT);  
	pinMode(rightEcho, INPUT);  

}

//---------------------------------------------------------------------
//----------------------Main loop function-----------------------------
//---------------------------------------------------------------------

void loop() { //NEED OPTIMIZATION
  decide();
}

//---------------------------------------------------------------------
//--------------Function to read sensor distances----------------------
//---------------------------------------------------------------------

void readSensors() {
  lDist = readDistance(leftTrig, leftEcho);
  fDist = readDistance(frontTrig, frontEcho);
  rDist = readDistance(rightTrig, rightEcho);
}

//---------------------------------------------------------------------
//----------Function to measure distances with sensor------------------
//---------------------------------------------------------------------

int readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(4);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  return pulseIn(echoPin, HIGH)/58;
}

//---------------------------------------------------------------------
//----------------Function to stop all motors--------------------------
//---------------------------------------------------------------------

void stopMotors() {
  digitalWrite(LEFT_MOTOR_ENABLE, LOW);
  digitalWrite(RIGHT_MOTOR_ENABLE, LOW);
}

//---------------------------------------------------------------------
//-Function to calculate the mean distance from left and right sensors-
//---------------------------------------------------------------------

void stabilize() {
  mean = (lDist + rDist) / 2;

  int desiredMean = 4;
  int error = mean - desiredMean;

  if (error > 0) {
    LmotorSpeed = baseSpeed - error;
    RmotorSpeed = baseSpeed + error;
  } else if (error < 0) {
    RmotorSpeed = baseSpeed + error;
    LmotorSpeed = baseSpeed - error;
  } else {
    LmotorSpeed = baseSpeed;
    RmotorSpeed = baseSpeed;
  }
}

//---------------------------------------------------------------------
//-----------------Function to move forward----------------------------
//---------------------------------------------------------------------

void goForward() {
  digitalWrite(LEFT_MOTOR_IN1, HIGH);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  digitalWrite(RIGHT_MOTOR_IN1, HIGH);
  digitalWrite(RIGHT_MOTOR_IN2, LOW);
  analogWrite(LEFT_MOTOR_ENABLE, 225);
  analogWrite(RIGHT_MOTOR_ENABLE, 225);
  Serial.println("Going forward");
  delay(goForwardTime);
}

//---------------------------------------------------------------------
//------------------Function to turn left------------------------------
//---------------------------------------------------------------------

void lTurn() {
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, HIGH);
  digitalWrite(RIGHT_MOTOR_IN1, HIGH);
  digitalWrite(RIGHT_MOTOR_IN2, LOW);
  analogWrite(LEFT_MOTOR_ENABLE, 180);
  analogWrite(RIGHT_MOTOR_ENABLE, 180);
  Serial.println("Turning left");
  delay(turnTime);
}

//---------------------------------------------------------------------
//------------------Function to turn right-----------------------------
//---------------------------------------------------------------------

void rTurn() {
  digitalWrite(LEFT_MOTOR_IN1, HIGH);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  digitalWrite(RIGHT_MOTOR_IN2, HIGH);
  analogWrite(LEFT_MOTOR_ENABLE, 180); 
  analogWrite(RIGHT_MOTOR_ENABLE, 180);
  Serial.println("Turning right");
  delay(turnTime);
}

//---------------------------------------------------------------------
//----------------Function to perform a u-turn-------------------------
//---------------------------------------------------------------------

void uTurn() {
  stopMotors();

  digitalWrite(LEFT_MOTOR_IN1, HIGH);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  digitalWrite(RIGHT_MOTOR_IN2, HIGH);
  analogWrite(LEFT_MOTOR_ENABLE, 200); 
  analogWrite(RIGHT_MOTOR_ENABLE, 200);
  delay(uTurnTime);

  stopMotors();
  Serial.println("Turning u");

  
}

//---------------------------------------------------------------------
//-----Function to check if the robot is at the goal position----------
//---------------------------------------------------------------------

bool isAtGoal(){ // needs to be implemented
  if((lDist>=15 && lDist<=30) && (rDist>=15 && rDist<=30)){
    Serial.println("We are on the objective");
    return true;
  }
  else{
    return false;
  }
}

//---------------------------------------------------------------------
//--------Function to make decisions based on sensor readings----------
//---------------------------------------------------------------------

void decide() {
  // Perform obstacle detection
  int frontDistance = sonarFront.ping_cm();
  int leftDistance = sonarLeft.ping_cm();
  int rightDistance = sonarRight.ping_cm();

  // Adjust behavior based on sensor readings
  if (frontDistance > 0 && frontDistance <= 20) { // If obstacle detected in front
    if (leftDistance > 0 && leftDistance <= 20) { // If obstacle detected on left
      // Turn right
      rTurn();
    } else if (rightDistance > 0 && rightDistance <= 20) { // If obstacle detected on right
      // Turn left
      lTurn();
    } else { // If no obstacles on left or right
      // Turn right
      rTurn();
    }
  } else { // No obstacle detected in front
    // Move forward
    goForward();
  }
}

//---------------------------------------------------------------------
//---Alternative function to make decisions based on sensor readings---
//---------------------------------------------------------------------

void decide2() { // may need optimization
    if (isAtGoal()) {
      stopMotors();
  } else if (fDist > fThershold) {
      goForward();
  } else {
      uTurn();

    if (rDist > thershold) {
      rTurn();
    }else if (lDist > thershold) {
      lTurn();
    }else {
      uTurn();
    }
  }
}

//---------------------------------------------------------------------
//---Alternative function to make decisions based on sensor readings---
//---------------------------------------------------------------------

void decide3() { //NEED OPTIMIZATION
    if (lDist > thershold) {
      lTurn();
  } else if (rDist > thershold) {
      rTurn();
  } else if (fDist > fThershold) {
      goForward();
  } else {
      uTurn();
  }
}