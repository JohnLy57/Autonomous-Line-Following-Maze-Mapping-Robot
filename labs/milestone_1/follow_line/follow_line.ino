/*
  Program that enables robot to move along a straight line.
  
  The main loop allows the robot to periodically check whether it deviates from
  the course and change its wheels' orientation and speed accordingly, aided by
  the IR sensor data.
  
  Once activated, the robot will be able to self-correct its trajectory along
  the white line, in real time and without manual interference.
  
*/

#include <Servo.h>

#define slow_R  88
#define slow_L  92
#define med_R   85
#define med_L   95
#define fast_R  0
#define fast_L  180

Servo left;
Servo right;
int pinLeft = 5;
int pinRight = 6;
int lineSensorL = A0;
int lineSensorM = A1;
int lineSensorR = A2;
int LS1;
int LS2;
int LS3;


void leftForward() {
  left.write(fast_L);
}

void rightForward() {
  right.write(fast_R);
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

void robotStop() {
  leftStop();
  rightStop();
  delay(2000);
}

void followLine() {
  if (LS1 > 400&&LS3 < 650){// 
    right.write(slow_R);//slow down left wheel  
    Serial.print("test");  
  }
  else if (LS1 < 400 && LS3 > 650){
    left.write(slow_L);//slow down right wheel
  }
  else robotForward();
}

void setup() {
  //Pin Setup
  Serial.begin(9600);
  left.attach(pinLeft);
  right.attach(pinRight);

  //Motor Setup
  leftStop();
  rightStop();
  robotForward();
}

void loop() {
  LS1 = analogRead(lineSensorL);
  LS2 = analogRead(lineSensorM);
  LS3 = analogRead(lineSensorR);
  
  /*
  // debugging code
  Serial.print(LS1);
  Serial.print(" ");
  Serial.print(LS2);
  Serial.print(" ");
  Serial.println(LS3);
  */

  followLine();
}
