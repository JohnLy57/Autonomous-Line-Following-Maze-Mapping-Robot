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

#define back_slow_R 92
#define back_slow_L 88
#define back_med_R  95
#define back_med_L  85
#define back_fast_R 180 
#define back_fast_L 0

#define left_to  500
#define mid_to   500
#define right_to 500

Servo left;
Servo right;
int pinLeft = 5;
int pinRight = 6;
int lineSensorL = A3;
int lineSensorM = A5;
int lineSensorR = A4;
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
  leftForward(slow_L);
  rightForward(slow_R);
}

void robotStop() {
  leftStop();
  rightStop();
  //delay(200);
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

void pd_followLine() {
  //working slow settings:
  // 0.9, 85 +/- 5
  
  
  int KD = 0.9;
  int tempspeed;
  
  checkSensors();
  // right drifts onto white
  if (LSR < 400){
    int rdiff = (850 - LSL)/850;
    int tempspeed = 87+rdiff*KD*3;
    right.write(tempspeed);
  }
  else if (LSL < 400) {
    int ldiff = (800 - LSL)/800;
    int tempspeed = 93-ldiff*KD*3;
    left.write(tempspeed);
  }
  else{
    leftForward(slow_L);
    rightForward(slow_R);
  }
}

void checkIntersect() {
  if (LSL < left_to && LSR < right_to){
    intersectFlag = 1;
  }
}

void turnLeft() {
    delay(250);
    robotStop();
    rightForward(slow_R);   
    leftForward(back_slow_L);    
     delay(200);

    int ready = 0;
    while(ready != 2){
      ready += checkLine();
      delay(10);
    }
    intersectFlag = 0;
}

void turnRight() {
    delay(250);
    robotStop();
    leftForward(slow_L);   
    rightForward(back_slow_R);    
        delay(200);

    int ready = 0;
    while(ready != 2){
      ready += checkLine();
      delay(10);
    }
    intersectFlag = 0;
}

void turn(int direction){
  if (!direction){
    turnLeft();
  }
  else turnRight();
}
  
int checkLine() {
  checkSensors();
  if (LSL > left_to && LSM <mid_to && LSR > right_to){
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

void spitprint() {
  int LS1 = analogRead(lineSensorL);
  int LS2 = analogRead(lineSensorM);
  int LS3 = analogRead(lineSensorR);
  
  
  // debugging code
  Serial.print(LS1);
  Serial.print(" ");
  Serial.print(LS2);
  Serial.print(" ");
  Serial.println(LS3);
  
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

void pd_eight() {
  int leftTurns = 0;
  int rightTurns = 0;
  
  while(1){
    pd_followLine();
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
      pd_followLine();
    }
  }  
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
  pd_eight();
  //followLine();
  //checkIntersect();
  //if(intersectFlag){
   //turnLeft();
  //}
  //spitprint();
  //pd_followLine();
}
