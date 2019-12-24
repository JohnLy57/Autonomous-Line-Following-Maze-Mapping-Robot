/*
    Basic functions for robot control. 
    The main loop has the robot move forward for 5 seconds, stop for 5 seconds, move backward for 5 seconds, stop for 5 seconds and repeat.
    This allows for determining if the servos were calibrated correctly and if the robot is driving straight.
*/

#include <Servo.h>

Servo left;
Servo right;
int pinLeft = 5;
int pinRight = 6;

void leftBackward() {
  left.write(0);
}

void leftForward() {
  left.write(180);
}

void rightForward() {
  right.write(0);
}

void rightBackward() {
  right.write(180);
}

void leftStop() {
  left.write(90);
}

void rightStop() {
  right.write(90);
}

void robotForward() {
  leftForward();
  rightForward();
}

void robotBackward() {
  leftBackward();
  rightBackward();
}

void robotStop() {
  leftStop();
  rightStop();
}

void setup() {
  left.attach(pinLeft);
  right.attach(pinRight);
  leftStop();
  rightStop();
}

void loop() { 
  robotForward();   //Drives robot forward 
  delay(5000);
  robotStop();      //Stops robot
  delay(5000);
  robotBackward();  //Drives robot backward
  delay(5000);
  robotStop();      //Stops robot
  delay(1000);
}