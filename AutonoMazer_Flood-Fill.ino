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

#define MAZE_SIZE_X 20
#define MAZE_SIZE_Y 20

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
//---------------------Flood-fill algorithms---------------------------
//---------------------------------------------------------------------

bool visited[MAZE_SIZE_X][MAZE_SIZE_Y] = {false};

bool isVisited(int x, int y) {
  // Check if the coordinates are within the maze boundaries
  if (x >= 0 && x < MAZE_SIZE_X && y >= 0 && y < MAZE_SIZE_Y) {
    return visited[x][y];
  } else {
    // If coordinates are outside maze boundaries, consider it visited
    return true;
  }
}

// Function to mark a cell at coordinates (x, y) as visited
void markVisited(int x, int y) {
  // Check if the coordinates are within the maze boundaries
  if (x >= 0 && x < MAZE_SIZE_X && y >= 0 && y < MAZE_SIZE_Y) {
    visited[x][y] = true;
  }
}

bool isLeftCellFree(int x, int y) {
  return (x > 0 && !isObstacle(x - 1, y)); // Assuming isObstacle function checks if there's an obstacle at the given coordinates
}

bool isFrontCellFree(int x, int y) {
  return (y < MAZE_SIZE_Y - 1 && !isObstacle(x, y + 1)); // Assuming isObstacle function checks if there's an obstacle at the given coordinates
}

bool isRightCellFree(int x, int y) {
  return (x < MAZE_SIZE_X - 1 && !isObstacle(x + 1, y)); // Assuming isObstacle function checks if there's an obstacle at the given coordinates
}

// Function to check if there is an obstacle at coordinates (x, y)
bool isObstacle(int x, int y) {
 
  const int obstacleThreshold = 8; // Example threshold value
  
  // Check the sensor readings to determine if there is an obstacle
  if (x >= 0 && x < MAZE_SIZE_X && y >= 0 && y < MAZE_SIZE_Y) {
    // Check the distance readings from left, front, and right sensors
    if (lDist <= obstacleThreshold && y > 0) {
      return true; // Obstacle detected on the left side
    }
    if (fDist <= obstacleThreshold && x < MAZE_SIZE_X - 1) {
      return true; // Obstacle detected in front
    }
    if (rDist <= obstacleThreshold && y < MAZE_SIZE_Y - 1) {
      return true; // Obstacle detected on the right side
    }
  }
  // No obstacle detected at the given coordinates
  return false;
}


//---------------------------------------------------------------------
//----------------------Main loop function-----------------------------
//---------------------------------------------------------------------

void loop() {
  readSensors(); // Read sensor data
  
  if (isAtObjective()) {
    stopMotors(); // Stop if the goal is reached
    Serial.println("Goal reached!");
    while (true); // Stay in a loop forever
  }
  
  // Check if the current cell has been visited
  if (!isVisited()) {
    markVisited(); // Mark the current cell as visited
  }
  
  // Explore neighboring cells
  if (isLeftCellFree()) {
    lTurn(); // Turn left if the left cell is free
  } else if (isFrontCellFree()) {
    goForward(); // Go forward if the front cell is free
  } else if (isRightCellFree()) {
    rTurn(); // Turn right if the right cell is free
  } else {
    uTurn(); // Make a U-turn if no free cells are available
  }
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
  delayMicroseconds(2);
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

bool isAtObjective(){ 
  int objectiveWidth = 40; // in centimeters
  int objectiveHeight = 40; // in centimeters
  int robotWidth = 12; // in centimeters

  // Calculate the distance from left and right walls of the objective area
  int leftDistanceFromObjective = lDist - (objectiveWidth / 2) + (robotWidth / 2);
  int rightDistanceFromObjective = rDist - (objectiveWidth / 2) + (robotWidth / 2);

  // Check if the robot is within the objective area
  if (leftDistanceFromObjective >= 0 && rightDistanceFromObjective >= 0 &&
      leftDistanceFromObjective <= objectiveWidth && rightDistanceFromObjective <= objectiveWidth) {
    Serial.println("We are on the objective");
    return true;
  } else {
    return false;
  }
}
