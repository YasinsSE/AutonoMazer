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

//---------------------------------------------------------------------
//---------------------Ultrasonic sensor pins--------------------------
//---------------------------------------------------------------------

const int lTrig = 0, fTrig = 2, rTrig = 4;
const int lEcho = 1, fEcho = 3, rEcho = 5;

//---------------------------------------------------------------------
//---------------------------Motor pins--------------------------------
//---------------------------------------------------------------------

const int lBack = 7 , lForward = 8 , rBack  = 9, rForward = 10; // check pins

int lDist, rDist, fDist;
int mean;
int sum;

int LmotorSpeed = baseSpeed, RmotorSpeed = baseSpeed;

//---------------------------------------------------------------------
//---Setup function to initialize serial communication and pin modes---
//---------------------------------------------------------------------

void setup() {
  Serial.begin(19200);
  pinMode(rBack, OUTPUT);
  pinMode(rForward, OUTPUT);
  pinMode(lBack, OUTPUT);
  pinMode(lForward, OUTPUT);

  pinMode(lTrig, OUTPUT);  
	pinMode(lEcho, INPUT);  
  pinMode(fTrig, OUTPUT);  
	pinMode(fEcho, INPUT);  
  pinMode(rTrig, OUTPUT);  
	pinMode(rEcho, INPUT);  

}

//---------------------------------------------------------------------
//----------------------Main loop function-----------------------------
//---------------------------------------------------------------------

void loop() { //NEED OPTIMIZATION
  readSensors();
	delay(50);  
  stabilize();
	delay(50);  
  decide2();
	delay(50);  
  stopAll();
	delay(50);  
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
  return pulseIn(echoPin, HIGH)/58;
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
  analogWrite(lForward, LmotorSpeed + lExtraSpeedSpeed);
  digitalWrite(lBack, LOW);
  analogWrite(rForward, RmotorSpeed);
  digitalWrite(rBack, LOW);
  Serial.println("Going forward");
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
  Serial.println("Turning left");
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
  Serial.println("Turning right");
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
  Serial.println("Turning u");
  delay(uTurnTime);
  
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

void decide() { // may need optimization
    if (isAtGoal()) {
      stopAll();
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
