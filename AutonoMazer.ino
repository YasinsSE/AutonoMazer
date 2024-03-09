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

//---------------------------------------------------------------------
//--------------Constants for motor speeds and timing------------------
//---------------------------------------------------------------------

// const int max_distance = 350;
const int baseSpeed = 150; // Need optimizations
const int lExtraSpeedSpeed = 30;
const int turnTime = 350;
const int uTurnTime = 750;
const int goForwardTime = 250;
const int thershold = 40;
const int fThershold = 20;
/*
const int errorMargin = 5;
const int pathWidth = 20;
const int targetWidth = 40 */

//---------------------------------------------------------------------
//---------------------Ultrasonic sensor pins--------------------------
//---------------------------------------------------------------------

const int lTrig = A0, fTrig = A1, rTrig = A2;
const int lEcho = A3, fEcho = A4, rEcho = A5;

//---------------------------------------------------------------------
//---------------------------Motor pins--------------------------------
//---------------------------------------------------------------------

const int lBack = 1 , lForward = 2 , rBack = 4, rForward = 5; // check pins

int lDist, rDist, fDist;
int mean;

int LmotorSpeed = baseSpeed, RmotorSpeed = baseSpeed;

//---------------------------------------------------------------------
//---Setup function to initialize serial communication and pin modes---
//---------------------------------------------------------------------

void setup() {
  Serial.begin(9600);
  pinMode(rBack, OUTPUT);
  pinMode(rForward, OUTPUT);
  pinMode(lBack, OUTPUT);
  pinMode(lForward, OUTPUT);
}

//---------------------------------------------------------------------
//----------------------Main loop function-----------------------------
//---------------------------------------------------------------------

void loop() { //NEED OPTIMIZATION
  readSensors();
  stabilize();
  decide();
  stopAll();
}

//---------------------------------------------------------------------
//--------------Function to read sensor distances----------------------
//---------------------------------------------------------------------

void readSensors() {
  lDist = readDistance(lTrig, lEcho);
  fDist = readDistance(fTrig, fEcho);
  rDist = readDistance(rTrig, rEcho);
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
  return pulseIn(echoPin, HIGH) / 58;
}

//---------------------------------------------------------------------
//----------------Function to stop all motors--------------------------
//---------------------------------------------------------------------

void stopAll() {
  digitalWrite(rBack, LOW);
  digitalWrite(rForward, LOW);
  digitalWrite(lBack, LOW);
  digitalWrite(lForward, LOW);
}

//---------------------------------------------------------------------
//-Function to calculate the mean distance from left and right sensors-
//---------------------------------------------------------------------

void stabilize() { //NEED OPTIMIZATION
  mean = (lDist + rDist) / 2;
}

//---------------------------------------------------------------------
//-----------------Function to move forward----------------------------
//---------------------------------------------------------------------

void goForward() {
  analogWrite(lForward, LmotorSpeed + lExtraSpeedSpeed);
  digitalWrite(lBack, LOW);
  analogWrite(rForward, RmotorSpeed);
  digitalWrite(rBack, LOW);
  delay(goForwardTime);
}

//---------------------------------------------------------------------
//------------------Function to turn left------------------------------
//---------------------------------------------------------------------

void lTurn() {
  digitalWrite(lForward, LOW);
  digitalWrite(lBack, HIGH);
  digitalWrite(rForward, HIGH);
  digitalWrite(rBack, LOW);
  delay(turnTime);
}

//---------------------------------------------------------------------
//------------------Function to turn right-----------------------------
//---------------------------------------------------------------------

void rTurn() {
  digitalWrite(rForward, LOW);
  digitalWrite(rBack, HIGH);
  digitalWrite(lForward, HIGH);
  digitalWrite(lBack, LOW);
  delay(turnTime);
}

//---------------------------------------------------------------------
//----------------Function to perform a u-turn-------------------------
//---------------------------------------------------------------------

void uTurn() {
  digitalWrite(lForward, LOW);
  digitalWrite(lBack, HIGH);
  digitalWrite(rForward, HIGH);
  digitalWrite(rBack, LOW);
  delay(uTurnTime);
}

//---------------------------------------------------------------------
//-----Function to check if the robot is at the goal position----------
//---------------------------------------------------------------------

bool isAtGoal(){ // needs to be implemented
  
}

//---------------------------------------------------------------------
//--------Function to make decisions based on sensor readings----------
//---------------------------------------------------------------------

void decide() { // may need optimization
  if (isAtGoal()) {
    stopAll();
  } else if (fDist > fThershold) {
    goForward();
  } else {
    uTurn();

    if (rDist > thershold) {
      rTurn();
    } else if (lDist > thershold) {
      lTurn();
    } else {
      uTurn();
    }
  }
}

//---------------------------------------------------------------------
//---Alternative function to make decisions based on sensor readings---
//---------------------------------------------------------------------

void decide2() { //NEED OPTIMIZATION
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
