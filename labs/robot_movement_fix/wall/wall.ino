#include <Servo.h>


#define slow_R 88
#define slow_L 92
#define med_R 87
#define med_L 93
#define med_fast_R 85
#define med_fast_L 95
#define fast_R 0
#define fast_L 180

#define back_slow_R 92
#define back_slow_L 88
#define back_med_R 93
#define back_med_L 87
#define back_med_fast_R 95
#define back_med_fast_L 85
#define back_fast_R 180
#define back_fast_L 0

#define left_to 500
#define mid_to 500
#define right_to 500

// dist sensors thresholds
#define distLow 180 //far
#define distHigh 600 //close

#define frontDistLow 180 //far
#define frontDistHigh 210 //close

int sampler = 5;
int sampler2 = 5; //8

int start = 0;

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
int turnDirection = 0; // 0 is left, 1 is right.


// dist sensors
int common_dist_pin = A3;
int distFront = 0;
int distBack = 0;
int distRight = 0;

//  IR sensors
int irFront = 0;
int irBack = 0;
int irLeft = 0;
int irRight = 0;

// mux control pins
int mux_a = 7;
int mux_b = 8;
int mux_c = 9;

int ledPin_front = 2;
int ledPin_right = 3;

void leftForward(int speed)
{
  left.write(speed);
}

void rightForward(int speed)
{
  right.write(speed);
}

void leftStop()
{
  left.write(90);
}

void rightStop()
{
  right.write(90);
}

void robotForward()
{
  leftForward(med_fast_L);
  rightForward(med_fast_R);
}

void robotStop()
{
  leftStop();
  rightStop();
  //delay(200);
}

void pd_followLine()
{
  //working slow settings:
  // 0.9, 85 +/- 5

  int KD = 0.9;
  int tempspeed;

  checkSensors();
  // right drifts onto white
  if (LSR < 500)
  {
    int rdiff = (850 - LSR) / 850;
    int tempspeed = 87 + rdiff * KD * 3;
    right.write(tempspeed);
  }
  else if (LSL < 500)
  {
    int ldiff = (800 - LSL) / 800;
    int tempspeed = 93 - ldiff * KD * 3;
    left.write(tempspeed);
  }
  else
  {
    leftForward(med_fast_L);
    rightForward(med_fast_R);
  }
}

/* Check whether the robot is at an intersect; 
   set intersectFlag = 1 if true */
void checkIntersect()
{
  checkSensors();
  if (LSL < left_to && LSR < right_to)
  {
    intersectFlag = 1;

  }
}

void pivotLeft() 
{
  rightForward(med_R);
  leftForward(med_L);
  delay(300);
  robotStop();
  rightForward(med_R);
  leftForward(back_med_L);
  delay(300);

  int ready = 0;
  while (ready != sampler2)
  {
    ready += checkLine();
  }
  intersectFlag = 0;
}

void turnLeft()
{
  delay(60);
  robotStop();
  rightForward(med_R);
  leftForward(back_slow_L);
  delay(50);

  int ready = 0;
  while (ready != sampler2)
  {
    ready += checkLine();
  }
  intersectFlag = 0;
}

void pivotRight() 
{
  rightForward(med_R);
  leftForward(med_L);
  delay(300);
  robotStop();
  leftForward(med_L);
  rightForward(back_med_R);
  delay(300);

  int ready = 0;
  while (ready != sampler2)
  {
    ready += checkLine();
  }
  intersectFlag = 0;
}

void turnRight()
{
  delay(60);
  robotStop();
  leftForward(med_L);
  rightForward(back_slow_R);
  delay(50);

  int ready = 0;
  while (ready != sampler2)
  {
    ready += checkLine();
  }
  //delay(50);
  intersectFlag = 0;
}

void turn(int direction)
{
  if (!direction)
  {
    turnLeft();
  }
  else
    turnRight();
}

/* Check whether robot is on a straight line;
   Return 1 if true, 0 if false */
int checkLine()
{
  checkSensors();
  if (LSL > left_to && LSM < mid_to && LSR > right_to)
  {
    return 1;
  }
  else
    return 0;
}

/* Get the averaged readings from line sensors (for line following) */
void checkSensors()
{
  int av_L = 0;
  int av_R = 0;
  int av_M = 0;
  int sampler = 4;
  
  for (int x = 0; x < sampler; x++)
  {
    av_L += analogRead(lineSensorL);
    av_M += analogRead(lineSensorM);
    av_R += analogRead(lineSensorR);
  }

  LSL = av_L / sampler;
  LSM = av_M / sampler;
  LSR = av_R / sampler;
}

/* Get the readings from distance sensors (for wall detection)*/
void checkDistSensors() {
  int ready = 0;
  while(ready<sampler){

  // SWEEP PIN:
  // FRONT MUX = 2
  // RIGHT MUX = 1
  // BACK MUX =  0
  digitalWrite(mux_a, LOW);       
  digitalWrite(mux_b, HIGH);       
  digitalWrite(mux_c, LOW);   
  distFront += analogRead(A3);
  digitalWrite(mux_a, LOW);       
  digitalWrite(mux_b, LOW);       
  digitalWrite(mux_c, LOW);   
  distBack += analogRead(A3);
  digitalWrite(mux_a, HIGH);       
  digitalWrite(mux_b, LOW);       
  digitalWrite(mux_c, LOW);   
  distRight += analogRead(A3);
  ready +=1;
  }
  distFront = distFront/sampler;
  distBack = distBack/sampler;
  distRight = distRight/sampler;
}

/* Get the readings from IR sensors (for robot detection)*/
void checkIR(){
  // FRONT MUX = 4 
  // RIGHT MUX = 6  
  // LEFT MUX = 7  
  // AFT MUX = 5 
  digitalWrite(mux_a, LOW);       
  digitalWrite(mux_b, LOW);       
  digitalWrite(mux_c, HIGH);
  irFront = analogRead(A3);
  digitalWrite(mux_a, LOW);       
  digitalWrite(mux_b, HIGH);       
  digitalWrite(mux_c, HIGH);
  irRight = analogRead(A3);
  digitalWrite(mux_a, HIGH);       
  digitalWrite(mux_b, HIGH);       
  digitalWrite(mux_c, HIGH);
  irLeft = analogRead(A3);
  digitalWrite(mux_a, HIGH);       
  digitalWrite(mux_b, LOW);       
  digitalWrite(mux_c, HIGH);
  irBack = analogRead(A3);
}

/* Check if there are walls around the robot; make it react accordingly */
void checkWall() {
  
  checkDistSensors();
//
//    Serial.println(distFront); 
//    Serial.println(distRight);
//    Serial.println(distBack);
  // all walls
  if((distRight<distHigh && distRight>distLow) && (distFront>frontDistHigh) && (distBack<distHigh && distBack>distLow)){
    pivot180();
  }
  
  // wall on right and in front
  else if ((distRight < distHigh && distRight > distLow) && (distFront>frontDistHigh)) {
    //rotate left 90
    digitalWrite(ledPin_front, HIGH);
    digitalWrite(ledPin_right, HIGH);
    pivotLeft();
    digitalWrite(ledPin_front, LOW);
    digitalWrite(ledPin_right, LOW);
  }

  // no wall infront, drive forward.
  else if ((distRight < distHigh && distRight > distLow) && (distFront<frontDistHigh)) { 
    //go straight
    digitalWrite(ledPin_right, HIGH);
    pd_followLine();
    digitalWrite(ledPin_right, LOW);
  }

  //no wall on right and wall in front
  else if ((distRight > distHigh || distRight < distLow) && (distFront>frontDistHigh)){
    //take a right turn
    digitalWrite(ledPin_front, HIGH);
        pivotRight();
        digitalWrite(ledPin_front, LOW);
  }
  // no wall on right, no wall in front
  else{
    //go forward
    pd_followLine();
  }
}

void pivot180(){
    leftForward(med_L);
    rightForward(back_med_fast_R);
    delay(200);
    int ready = 0;
    while (ready != 5)
    {
      ready += checkLine();
    }
    intersectFlag=0;
}

void spinner(){
    pd_followLine();
    checkIntersect();
    if(intersectFlag){
      pivot180();
    }
}

void setup()
{
  //Pin Setup
  Serial.begin(9600);
  left.attach(pinLeft);
  right.attach(pinRight);
  pinMode(mux_a, OUTPUT);
  pinMode(mux_b, OUTPUT);
  pinMode(mux_c, OUTPUT);
  pinMode(ledPin_right, OUTPUT);
  pinMode(ledPin_front, OUTPUT);
  pinMode(13, INPUT);
  
  //Motor Setup
  leftStop();
  rightStop();
  //robotForward();
}

void loop()
{
    if (digitalRead(13) == 1){
      start = 1;
      delay(250);
    }
  
    if(start){
      //spinner();
      while(1) {
        checkIntersect();
        pd_followLine();
        if(intersectFlag) {
          checkWall();
          intersectFlag = 0;
        }
      }
    }
//  while(1){
//    checkDistSensors();
//    //Serial.println(distFront); 
//    //Serial.println(distRight);
//    Serial.println(distBack);
//    delay(200);
//  }

}
