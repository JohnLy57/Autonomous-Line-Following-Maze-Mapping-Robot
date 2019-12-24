/*
  Program that enables robot to move along a straight line.
  
  The main loop allows the robot to periodically check whether it deviates from
  the course and change its wheels' orientation and speed accordingly, aided by
  the IR sensor data.
  
  Once activated, the robot will be able to self-correct its trajectory along
  the white line, in real time and without manual interference.

  ---------------------------------------------------------------------------

  White is low, Black is high.
  THRESHOLDS:
  LSL: 400
  LSR: 650
*/

#include <Servo.h>

#define slow_R  88
#define slow_L  92
#define med_R   85
#define med_L   95
#define fast_R  0
#define fast_L  180

#define back_slow_L 
#define back_slow_R
#define back_med_L 
#define back_med_R
#define back_fast_L 
#define back_fast_R

#define left_to  400
#define mid_to   400
#define right_to 650

Servo left;
Servo right;
int pinLeft = 5;
int pinRight = 6;
int lineSensorL = A0;
int lineSensorM = A1;
int lineSensorR = A2;
int LSL;
int LSM;
int LSR;
int intersectFlag = 0;
int turnDirection = 0;  // 0 is left, 1 is right.

void leftForward(int speed) {
  left.write(speed);
}

void rightForward(int speed) {
  right.write(speed);
}

void leftStop() {
  left.write(90);
}

void rightStop() {
  right.write(90);
}

void robotForward() {
  leftForward(fast_L);
  rightForward(fast_R);
}

void robotStop() {
  leftStop();
  rightStop();
  delay(200);
}

void followLine() {
  checkSensors();
  robotForward();
  if (LSL > left_to && LSR < right_to){// 
    right.write(slow_R);//slow down left wheel  
  }
  else if (LSL < left_to && LSR > right_to){
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
  //robotForward();
}

void checkIntersect() {
  if (LSL < left_to && LSR < right_to){
    intersectFlag = 1;
  }
}

void turnLeft() {
    robotStop();
    rightForward(fast_R);

    delay(1000);
    
    int ready = 0;
    while(!ready){
      ready = checkLine();
    }
    intersectFlag = 0;
    delay(150);

}

void turnRight() {
    robotStop();
    leftForward(fast_L);
    
    delay(1000);
    
    int ready = 0;
    while(!ready){
      ready = checkLine();
    }
    intersectFlag = 0;
    delay(150);

}

void turn(int direction){
  if (!direction){
    turnLeft();
  }
  else turnRight();
}
  
int checkLine() {
  checkSensors();
  if (LSL > left_to && LSR > right_to){
    return 1;
  }
  else return 0;
}

void checkSensors () {
  int av_L = 0;
  int av_R = 0;
  int av_M = 0;

  for (int x=0;x<5;x++){
    av_L += analogRead(lineSensorL);
    av_M += analogRead(lineSensorM);
    av_R += analogRead(lineSensorR);
  }
  
  LSL = av_L/5;
  LSM = av_M/5;
  LSR = av_R/5;
}

void eight() {
  int leftTurns = 0;
  int rightTurns = 0;
  
  while(1){
    followLine();
    checkIntersect();
    if(intersectFlag){
      turn(turnDirection);
      if(turnDirection == 0 && leftTurns<4){
        leftTurns +=1;
      }
      else if (turnDirection == 1 && rightTurns<4){
        rightTurns +=1;
      }
      if(leftTurns == 4){
        turnDirection = 1;
        leftTurns = 0;
      }
      else if(rightTurns == 4){
        turnDirection = 0;
        rightTurns = 0;
      }
      followLine();
    }
  }  
}

void loop() {
  //eight();
  followLine();
  checkIntersect();
  if(intersectFlag){
   turnLeft();
  }
}
