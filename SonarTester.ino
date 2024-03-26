// This uses Serial Monitor to display Range Finder distance readings
// Include NewPing Library
#include "NewPing.h"
// Hook up HC-SR04 with Trig to Arduino Pin 9, Echo to Arduino pin 10
//#define TRIGGER_PIN 9
//#define ECHO_PIN 10
#define leftTrig 0
#define leftEcho 1
#define frontTrig 2
#define frontEcho 3
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
  if (frontDistance >= 400 || frontDistance <= 2) 
  {
    Serial.println("Out of range");
  }
  else 
  {
    Serial.print(frontDistance);
    Serial.println(" cm -front-\n");
  }

  Serial.print("Distance to left = ");
  if (leftDistance >= 400 || leftDistance <= 2) 
  {
    Serial.println("Out of range");
  }
  else 
  {
    Serial.print(leftDistance);
    Serial.println(" cm -left-\n");
  }

  Serial.print("Distance to right = ");
  if (rightDistance >= 400 || rightDistance <= 2) 
  {
    Serial.println("Out of range");
  }
  else 
  {
    Serial.print(rightDistance);
    Serial.println(" cm -right-\n");
  }
  delay(500);
}