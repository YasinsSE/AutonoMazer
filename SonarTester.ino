// This program tests 3 ultrasonic sensors to evaluate their measurement accuracy and determine the distances they measure.
// This uses Serial Monitor to display Range Finder distance readings
#include "NewPing.h"
// HC-SR04
#define leftTrig 2
#define leftEcho 3
#define frontTrig 8
#define frontEcho 11
#define rightTrig 4
#define rightEcho 5
// Maximum distance we want to ping for (in centimeters).
#define MAX_DISTANCE 350  
// NewPing setup of pins and maximum distance.
//NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
NewPing sonarLeft(leftTrig, leftEcho, MAX_DISTANCE);
NewPing sonarFront(frontTrig, frontEcho, MAX_DISTANCE);
NewPing sonarRight(rightTrig, rightEcho, MAX_DISTANCE);
float duration, distance;
float leftDistance, rightDistance, frontDistance;

void setup() 
{
  Serial.begin(38400);
}
void loop() 
{
  // Send ping, get distance in cm
  //distance = sonar.ping_cm();
  leftDistance = sonarLeft.ping_cm();
  frontDistance = sonarFront.ping_cm();
  rightDistance = sonarRight.ping_cm();
  
  // Send results to Serial Monitor
  Serial.print("Distance to front = ");
  Serial.print(frontDistance);
  Serial.println(" cm -front-");

  Serial.print("Distance to left = ");
  Serial.print(leftDistance);
  Serial.println(" cm -left-");
  
  Serial.print("Distance to right = ");
  Serial.print(rightDistance);
  Serial.println(" cm -right-");

  Serial.println(" ----------------------------------------- ");
  delay(6000);
}
