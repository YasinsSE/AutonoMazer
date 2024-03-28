/**
 * @file AutonoMazer.ino
 * @brief Autonomous robot control program using ultrasonic sensors and Arduino Uno.
 * @author Yasin YILDIRIM
 * @date 04/2024
 * 
 * This program controls the movement of an autonomous robot using ultrasonic sensors
 * to measure distances to surrounding walls and navigate towards a goal position. It utilizes motor control
 * and sensor reading functions to make decisions based on sensor readings.
 */

#include <NewPing.h>

//---------------------------------------------------------------------
//---------------------Ultrasonic sensor pins--------------------------
//---------------------------------------------------------------------

#define leftTrig 2
#define leftEcho 3
#define frontTrig 8
#define frontEcho 11
#define rightTrig 4
#define rightEcho 5

#define MAX_DISTANCE 350

NewPing sonarLeft(leftTrig, leftEcho, MAX_DISTANCE);
NewPing sonarFront(frontTrig, frontEcho, MAX_DISTANCE);
NewPing sonarRight(rightTrig, rightEcho, MAX_DISTANCE);

//---------------------------------------------------------------------
//---------------------------Motor pins--------------------------------
//---------------------------------------------------------------------

#define LEFT_MOTOR_ENABLE 9
#define LEFT_MOTOR_IN1 6
#define LEFT_MOTOR_IN2 7
#define RIGHT_MOTOR_ENABLE 10
#define RIGHT_MOTOR_IN1 12
#define RIGHT_MOTOR_IN2 13

//---------------------------------------------------------------------
//--------------Constants for motor speeds and timing------------------
//---------------------------------------------------------------------

float leftDistance, rightDistance, frontDistance;
float duration;

const int baseSpeed = 100; // Need optimizations
const int uTurnSpeed = 100;
const int sideTurnSpeed = 100;
const int lExtraSpeedSpeed = 30;
const int rExtraSpeedSpeed = 30;
const int turnTime = 430;
const int uTurnTime = 800;
const int goForwardTime = 250;
// Time = Angle * Wheelbase/ Speed
/*
const int thershold = 10;
const int fThershold = 12;
const int errorMargin = 5;
const int pathWidth = 20;
const int targetWidth = 40 */


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

  /*
  pinMode(leftTrig, OUTPUT);  
	pinMode(leftEcho, INPUT);  
  pinMode(frontTrig, OUTPUT);  
	pinMode(frontEcho, INPUT);  
  pinMode(rightTrig, OUTPUT);  
	pinMode(rightEcho, INPUT);  
*/
}


//---------------------------------------------------------------------
//----------------------Main loop function-----------------------------
//---------------------------------------------------------------------

void loop() {
  stopMotors();
  readDistances(); // Read sensor data
  
  if(frontDistance > 20){
    while(frontDistance >=20){
    goForward();
    readDistances();
    if(frontDistance < 20){ 
    stopMotors();
    delay(100);
     }
    }
  }

  if(leftDistance > rightDistance && leftDistance >= 20){
    turnLeft();
    readDistances();

  }
  else if(rightDistance > leftDistance && rightDistance >= 20){
    turnRight();
    readDistances();
  }
  else if((leftDistance < 15) && (rightDistance < 15) && (frontDistance < 20)){
    uTurn();
    readDistances();
  }
  else{
    delay(100);
    turnLeft();
  }
  delay(250);
}



//---------------------------------------------------------------------
//----------Function to measure distances with sensor------------------
//---------------------------------------------------------------------

void readDistances() {
  leftDistance = sonarLeft.ping_cm();
  frontDistance = sonarFront.ping_cm();
  rightDistance = sonarRight.ping_cm();

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
  mean = (leftDistance + rightDistance) / 2;

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
  analogWrite(LEFT_MOTOR_ENABLE, baseSpeed);
  analogWrite(RIGHT_MOTOR_ENABLE, baseSpeed);
  Serial.println("Going forward");
  Serial.println(frontDistance);

}

//---------------------------------------------------------------------
//------------------Function to turn left------------------------------
//---------------------------------------------------------------------

void turnLeft() {
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, HIGH);
  digitalWrite(RIGHT_MOTOR_IN1, HIGH);
  digitalWrite(RIGHT_MOTOR_IN2, LOW);
  analogWrite(LEFT_MOTOR_ENABLE, sideTurnSpeed);
  analogWrite(RIGHT_MOTOR_ENABLE, sideTurnSpeed);
  Serial.println("Turning left");
  Serial.println(leftDistance);
  delay(turnTime);
  stopMotors();
}

//---------------------------------------------------------------------
//------------------Function to turn right-----------------------------
//---------------------------------------------------------------------

void turnRight() {
  digitalWrite(LEFT_MOTOR_IN1, HIGH);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  digitalWrite(RIGHT_MOTOR_IN2, HIGH);
  analogWrite(LEFT_MOTOR_ENABLE, sideTurnSpeed); 
  analogWrite(RIGHT_MOTOR_ENABLE, sideTurnSpeed);
  Serial.println("Turning right");
  Serial.println(rightDistance);
  delay(turnTime);
  stopMotors();
}

//---------------------------------------------------------------------
//----------------Function to perform a u-turn-------------------------
//---------------------------------------------------------------------

void uTurn() {
  stopMotors();
  delay(100);
  digitalWrite(LEFT_MOTOR_IN1, HIGH);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  digitalWrite(RIGHT_MOTOR_IN2, HIGH);
  analogWrite(LEFT_MOTOR_ENABLE, uTurnSpeed); 
  analogWrite(RIGHT_MOTOR_ENABLE, uTurnSpeed);
  delay(uTurnTime);
  stopMotors();
  Serial.println("Turning u");

  
}

//---------------------------------------------------------------------
//-----Function to check if the robot is at the goal position----------
//---------------------------------------------------------------------

bool isAtObjective(){ 
  int objectiveWidth = 40;
  int objectiveHeight = 40; 
  int robotWidth = 12; 

  // Calculate the distance from left and right walls of the objective area
  int leftDistanceFromObjective = leftDistance - (objectiveWidth / 2) + (robotWidth / 2);
  int rightDistanceFromObjective = rightDistance - (objectiveWidth / 2) + (robotWidth / 2);

  // Check if the robot is within the objective area
  if (leftDistanceFromObjective >= 0 && rightDistanceFromObjective >= 0 &&
      leftDistanceFromObjective <= objectiveWidth && rightDistanceFromObjective <= objectiveWidth) {
    Serial.println("We are on the objective");
    return true;
  } else {
    return false;
  }
}
