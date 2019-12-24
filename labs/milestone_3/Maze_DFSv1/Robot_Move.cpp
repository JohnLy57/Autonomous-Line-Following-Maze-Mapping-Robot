#include <Servo.h>



#define slow_R 88
#define slow_L 92
#define med_R 85
#define med_L 95
#define fast_R 0
#define fast_L 180

#define back_slow_R 92
#define back_slow_L 88
#define back_med_R 95
#define back_med_L 85
#define back_fast_R 180
#define back_fast_L 0

#define left_to 500
#define mid_to 500
#define right_to 500

// dist sensors thresholds
#define distLow 200 //far
#define distHigh 650 //close

Servo left;
Servo right;
int pinLeft = 5;
int pinRight = 6;

// line sensors
int lineSensorL = A0;
int lineSensorM = A1;
int lineSensorR = A2;
int LSL;
int LSM;
int LSR;
int intersectFlag = 0;


// dist sensors
int common_dist_pin = A3;
int distFront = 0;
int distLeft = 0;
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

int walls;
int xpos, ypos = 0;

////////////////////////////////////////////////
/*-------------FORWARD / BACK-----------------*/
////////////////////////////////////////////////

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
  leftForward(fast_L);
  rightForward(fast_R);
  switch(compass){
    case 0: //north
      ypos++;
      break;
    case 1: //east
      xpos++;
      break;
    case 2: //south
      ypos--;
      break;
    case 3: //west
      xpos--;
      break;
  }
}

void robotStop()
{
  leftStop();
  rightStop();
  //delay(200);
}

////////////////////////////////////////////////
/*------------------TURNING-------------------*/
////////////////////////////////////////////////

void pivotLeft() 
{
  delay(350);
  robotStop();
  rightForward(fast_R);
  leftForward(back_fast_L);
  delay(200);
  updateCompass(0);

  int ready = 0;
  while (ready != 2)
  {
    ready += checkLine();
    delay(10);
  }
  intersectFlag = 0;
}

void turnLeft()
{
  delay(250);
  robotStop();
  rightForward(fast_R);
  leftForward(back_slow_L);
  delay(200);
  updateCompass(0);

  int ready = 0;
  while (ready != 2)
  {
    ready += checkLine();
    delay(10);
  }
  intersectFlag = 0;
}

void pivotRight() 
{
  delay(350);
  robotStop();
  leftForward(fast_L);
  rightForward(back_fast_R);
  delay(200);
  updateCompass(1);

  int ready = 0;
  while (ready != 2)
  {
    ready += checkLine();
    delay(10);
  }
  intersectFlag = 0;
}

void turnRight()
{
  delay(250);
  robotStop();
  leftForward(fast_L);
  rightForward(back_slow_R);
  delay(200);

  updateCompass(1);

  int ready = 0;
  while (ready != 2)
  {
    ready += checkLine();
    delay(10);
  }
  intersectFlag = 0;
}


////////////////////////////////////////////////
/*---------------LINE FOLLOWING---------------*/
////////////////////////////////////////////////

void pd_followLine()
{
  //working slow settings:
  // 0.9, 85 +/- 5

  int KD = 0.9;
  int tempspeed;

  checkLineSensors();
  // right drifts onto white
  if (LSR < 400)
  {
    int rdiff = (850 - LSL) / 850;
    int tempspeed = 85 + rdiff * KD * 5;
    right.write(tempspeed);
  }
  else if (LSL < 400)
  {
    int ldiff = (800 - LSL) / 800;
    int tempspeed = 95 - ldiff * KD * 5;
    left.write(tempspeed);
  }
  else
  {
    robotForward();
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


////////////////////////////////////////////////
/*---------------SENSOR CHECKS----------------*/
////////////////////////////////////////////////

/* Check whether robot is on a straight line;
   Return 1 if true, 0 if false */
int checkLine()
{
  checkLineSensors();
  if (LSL > left_to && LSM < mid_to && LSR > right_to)
  {
    return 1;
  }
  else
    return 0;
}

/* Get the averaged readings from line sensors (for line following) */
void checkLineSensors()
{
  int av_L = 0;
  int av_R = 0;
  int av_M = 0;

  for (int x = 0; x < 5; x++)
  {
    av_L += analogRead(lineSensorL);
    av_M += analogRead(lineSensorM);
    av_R += analogRead(lineSensorR);
  }

  LSL = av_L / 5;
  LSM = av_M / 5;
  LSR = av_R / 5;
}

/* Get the readings from distance sensors (for wall detection)*/
void checkDistSensors() {
  // SWEEP PINS:
  // FRONT MUX = 2
  digitalWrite(mux_a, LOW);       
  digitalWrite(mux_b, HIGH);       
  digitalWrite(mux_c, LOW);   
  distFront = analogRead(A3);
  // Left MUX =  0
  digitalWrite(mux_a, LOW);       
  digitalWrite(mux_b, LOW);       
  digitalWrite(mux_c, LOW);   
  distLeft = analogRead(A3);
  // RIGHT MUX = 1
  digitalWrite(mux_a, HIGH);       
  digitalWrite(mux_b, LOW);       
  digitalWrite(mux_c, LOW);   
  distRight = analogRead(A3);
}


/* Get the readings from IR sensors (for robot detection)*/
void checkIRSensors(){
  // FRONT MUX = 4 
  digitalWrite(mux_a, LOW);       
  digitalWrite(mux_b, LOW);       
  digitalWrite(mux_c, HIGH);
  irFront = analogRead(A3);
  // RIGHT MUX = 6  
  digitalWrite(mux_a, LOW);       
  digitalWrite(mux_b, HIGH);       
  digitalWrite(mux_c, HIGH);
  irRight = analogRead(A3);
  // LEFT MUX = 7  
  digitalWrite(mux_a, HIGH);       
  digitalWrite(mux_b, HIGH);       
  digitalWrite(mux_c, HIGH);
  irLeft = analogRead(A3);
  // BACK MUX = 5 
  digitalWrite(mux_a, HIGH);       
  digitalWrite(mux_b, LOW);       
  digitalWrite(mux_c, HIGH);
  irBack = analogRead(A3);
}

/* Check if there are walls around the robot; make it react accordingly */
void checkWall() {
  checkDistSensors();
  walls = 0;
  //front sensor detects wall, set wall value with shift by compass to reorient
  if (distFront<distHigh && distFront>distLow){
    walls |= (0b1000>>compass); //set wall in front when facing north
  }
  //right sensor detects wall, set wall value with shift by compass to reorient
  if (distRight<distHigh && distRight>distLow){
    switch (compass){
      case 0: walls |= (0b0100); //wall is east
      case 1: walls |= (0b0010); //wall is south
      case 2: walls |= (0b0001); //wall is west
      case 3: walls |= (0b1000); //wall is north
    }
  }
  //left sensor detects wall, set wall value with shift by compass to reorient
  if (distLeft<distHigh && distLeft>distLow){
    switch (compass){
      case 0: walls |= (0b0001); //set wall on left when facing north
      case 1: walls |= (0b1000); //wall is north
      case 2: walls |= (0b0100); //wall is east
      case 3: walls |= (0b0010); //wall is south
    }
  }
}

/*Simple code for avoiding walls while moving in the maze
May be modified to include robot avoidance*/
void checkWallOld() {
  checkDistSensors();
  //wall on right, no wall in front
  if ((distRight < distHigh && distRight > distLow) && 
    (distFront>distHigh || distFront<distLow)) { 
    //go straight
    digitalWrite(ledPin_right, HIGH);
    pd_followLine();
    digitalWrite(ledPin_right, LOW);
  }
  //wall on right and in front
  else if ((distRight < distHigh && distRight > distLow) && (distFront<distHigh && distFront>distLow)) {
    //rotate left 90
    digitalWrite(ledPin_front, HIGH);
    digitalWrite(ledPin_right, HIGH);
    turnLeft();
    digitalWrite(ledPin_front, LOW);
    digitalWrite(ledPin_right, LOW);
  }
  //no wall on right and wall in front
  else if ((distRight > distHigh || distRight < distLow) && distFront<distHigh && distFront>distLow){
    //take a right turn
    digitalWrite(ledPin_front, HIGH);
        turnRight();
        digitalWrite(ledPin_front, LOW);
  }
  // no wall on right, no wall in front
  else{
    //go forward
    pd_followLine();
  }
}
