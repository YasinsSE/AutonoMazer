# Autonomous Robot Control

## Overview
This project contains the code for an autonomous robot that uses ultrasonic sensors to navigate its environment. The robot is controlled by an Arduino Uno and can move forward, turn left or right, and perform a u-turn based on sensor readings. The program makes decisions to avoid obstacles and navigate towards a goal position.

## Features
- **Obstacle Avoidance:** Uses three ultrasonic sensors to detect obstacles and navigate around them.
- **Motor Control:** Controls two motors to move the robot forward, turn left, turn right, and perform u-turns.
- **Serial Output:** Provides real-time feedback of the robot's actions and sensor readings through serial communication.

## Installation
1. **Open the project in the Arduino IDE:**
    - Open `main.ino` in the Arduino IDE.
2. **Install required libraries:**
    - Ensure you have the `NewPing` library installed. You can install it via the Library Manager in the Arduino IDE.


## Notes
- The program uses three ultrasonic sensors positioned on the left, front, and right sides of the robot.
- The motor control functions are designed for differential drive robots with two motors.


## Author
Yasin YILDIRIM

Feel free to open an issue if you have any questions or suggestions.
