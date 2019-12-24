//#include <Maze_DFS.c>
//#include <Robot_Move.c>
//#include <Servo.h>

#include "stdio.h"
#include "stdlib.h"
#include <Servo.h>

typedef uint8_t byte;

/**********************************************/
/*                   MAZE DFS                 */
/**********************************************/

#define n 100

//orientation of bot (0-3 = N-W)
enum Compass{
  North,
  East,
  South,
  West
};

typedef struct {
  int vertex;
  byte x;
  byte y;
  byte walls; //write to walls if unvisited node
  int adjNodes[4]; //nodes vertex values. 0-3 <=> N-W
  int back;
  // struct node *next
} node;

byte compass=0; //default north to start, bottom left corner
byte back; //direction behind bot;
byte diff = 0; //difference between compass and direction
int currentVertex = 0; //vertex 1-100
//default to (0,0), bottom left corner
int xpos, ypos = 0;

node *G[n];

/************** MAZE DFS - END ****************/


/* -------------------------------
 *           THRESHOLDS
 * -------------------------------
 */
//servo speeds
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

// line follower thresholds
#define left_to 500
#define mid_to 500
#define right_to 500

// dist sensors thresholds
#define distLow 200 //far
#define distHigh 650 //close

// turning thresholds
#define pivot_overshoot 250
#define turn_check_overshoot 200
#define check_line_delay 20

int sampler = 5;
int sampler2 = 8;

/* -------------------------------
 *        PIN DEFINITIONS
 * -------------------------------
 */
 
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

// light pins
int ledPin_front = 2;
int ledPin_right = 3;

byte walls;

/* -------------------------------
 *        MAIN INTEGRATION
 * -------------------------------
 */

char buffer[60];
int start = 0;

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
  
  DFSInit();
}

void loop()
{ 
  if (digitalRead(13) == 1){
    start ^= 1;
    delay(250);
    sprintf(buffer,"start: %d", start);
    Serial.println(buffer);
  }
//   checkWall();
  if (start){
//    pd_followLine();
//    checkIntersect();
//    if (intersectFlag){
//      checkWallOld();
//      pd_followLine();
//      intersectFlag=0;
//    }

     checkIntersect();
     if(intersectFlag) {
       DFS(0, 0);
       intersectFlag = 0;
     }
  }

}


////////////////////////////////////////////////
/*----------------- MAZE DFS -----------------*/
////////////////////////////////////////////////

//array containing values for adjacent verticies based on vertex numbers
// int adj[100][4]; 

/* Setup for DFS */
void DFSInit(){
    for(int i=0;i<n;i++){
        G[i]=NULL;
    }
}

/* Updates compass for turns. 0=left turn. 1=right turn. */
void updateCompass(int turn){
  // right turn
  if (turn){
    compass++;
    if (compass>3) compass=0; // W to N, 
  }
  // left turn
  if (!turn){
    compass--;
    if (compass<0) compass=3; // N to W,
  }
}

/* Updates currentVertex with respect to the grid, 0-99 */
void updateVertex(){
  currentVertex = xpos + ypos*10;
  sprintf(buffer,"currentVertex: %d", currentVertex);
  Serial.println(buffer);
}

void addBackNode(node currNode){
  switch(compass){
    case North: back = South;
      break;
    case East: back = West;
      break;
    case South: back = North;
      break;
    case West: back = East;
      break;
  }
}

/* Adds Vertex values of all adjacent nodes to current node.
Does not create new nodes. */
void addAllEdges(node *currNode){
  checkWall();
  //no wall north
  currNode->adjNodes[0] = (!(walls & 0b1000)) ? (currentVertex + 10) : -1;
  if (!(walls & 0b1000)) Serial.println("Path north");
  
  //no wall east
  currNode->adjNodes[1] = (!(walls & 0b0100)) ? (currentVertex + 1) : -1;
  if (!(walls & 0b0100)) Serial.println("Path east");

  //no wall south, needs work/ maybe a rear sensor
  // currNode->adjNodes[2] = (!(walls & 0b0010)) ? (currentVertex - 10) : -1;
  // if (!(walls & 0b0010)) Serial.println("Path south");
    currNode->adjNodes[2] = -1;


  //no wall west  
  currNode->adjNodes[3] = (!(walls & 0b0001)) ? (currentVertex - 1) : -1;
  if (!(walls & 0b0001)) Serial.println("Path west");

}

/* Changes direction to get to new edge using compass and direction. 
Moves to new node until intersect is found. */
void moveToNearbyNode(byte direction){
  // delay(1000);
  // Serial.println("MoveToNearbyNode Delay 1s");

  intersectFlag = 0;
  diff = abs(compass - direction);

  sprintf(buffer,"compass: %d, direction: %d", compass, direction);
  Serial.println(buffer);

  if (diff == 0){
    Serial.println("Turn: forward");
    pd_followLine();
    intersectFlag = 0;
  }

  else if (diff == 2) { //do a 180
    Serial.println("Turn: back");
    pivot180();
    pd_followLine();
  }
  else if (compass == (direction+1)) {
    Serial.println("Turn: left");
    pivotLeft();
    pd_followLine();
  }
  else if (compass == (direction-1)) {
    Serial.println("Turn: right");
    pivotRight();
    pd_followLine();
  }
  // sprintf(buffer,"moveToNearbyNode: compass: %d, direction: %d", compass, direction);
  // Serial.println(buffer);
  Serial.print("moveToNearbyNode: Compass:");
  Serial.print(compass);
  Serial.print(" Direction:");
  Serial.print(direction);
  Serial.println("");

  switch(compass){
    case North: //0
      ypos++;
      break;
    case East: //1
      xpos++;
      break;
    case South: //2
      ypos--;
      break;
    case West: //3
      xpos--;
      break;
  }
  // sprintf(buffer,"X: %d, Y: %d", xpos, ypos);
  // Serial.println(buffer);
  Serial.print("X:");
  Serial.print(xpos);
  Serial.print(" Y:");
  Serial.print(ypos);
  Serial.println("");

  while(!intersectFlag){
    pd_followLine();
    checkIntersect();
    // sprintf(buffer,"moveToNearbyNode: Intersect: %d", intersectFlag);
    // Serial.println(buffer);
    // Serial.print("moveToNearbyNode: Intersect:");
    Serial.print(intersectFlag);
    // Serial.println("");
  }
}

/* TODO: Outputs instructions for return path across visited nodes,
Bubble Up for DFS. */
void pathToNode(node target){
}


/* Depth First Search traversal on a starting point. 
DFS should be called at an intersection */
void DFS(int Vertex, byte direction)
{
  //printf("\n%d",i);
  // delay(1000);
  // Serial.println("Delay 1s");

  //DFS called upon new node, must move to node
  // robot do things

  //currently visiting a new node
  updateVertex();
  node *currNode;
  currNode=(node*)malloc(sizeof(node));
  currNode->vertex = currentVertex;
  currNode->x = xpos;
  currNode->y = ypos;
  currNode->walls = walls;
  currNode->back = back;
  addAllEdges(currNode);

  G[Vertex] = currNode;

  // Base case, while the adjacent neighbor of the current node is not NULL
  direction = 0;
  while(direction<4){
    Vertex = currNode->vertex;
    sprintf(buffer,"Direction: %d", direction);
    Serial.println(buffer);
    
    if(intersectFlag) {
      sprintf(buffer,"DFS Still Intersect: %d", intersectFlag);
       Serial.println(buffer);
      //idiot, you haven't visited anything yet   
      //choose new node to move to
      // delay(1000);
      // Serial.println("Delay 1s");

      
      // check for valid direction and if node in that direction is still unvisited (NULL)
      if ((G[Vertex]->adjNodes[direction]>-1) && (G[G[Vertex]->adjNodes[direction]]==NULL)){
        sprintf(buffer,"Direction: %d is NULL", direction);
        Serial.println(buffer);
        moveToNearbyNode(direction);
        DFS(Vertex, direction);
      }
      direction++;
    }
  }

  //backtrack to currNode first??
  //need return path damn...
  //calculate shortest path?  
  //visiting previous nodes now, not making new nodes 
  // pathToNode(G[currNode.vertex]);
  
}


/* -------------------------------
 *         MOTOR MOVEMENT 
 * -------------------------------
 */

/*
 * Description: Writes a speed to left motor.
 * @param speed: Pick from preset above. Use back value to reverse.
 */
void leftForward(int speed)
{
  left.write(speed);
}

/*
 * Description: Writes a speed to right motor.
 * @param speed: Pick from preset above. Use back value to reverse.
 */
 void rightForward(int speed)
{
  right.write(speed);
}

/*
 * Description: Stops the left motor.
 */
void leftStop()
{
  left.write(90);
}

/*
 * Description: Stops the right motor.
 */
void rightStop()
{
  right.write(90);
}

/*
 * Description: Starts both motors, in fast.
 */
void robotForward()
{
  leftForward(fast_L);
  rightForward(fast_R);
}


/*
 * Description: Stops both motors.
 */
void robotStop()
{
  leftStop();
  rightStop();
}

/* -------------------------------
 *            TURNING 
 * -------------------------------
 */


/*
 * Description: Pivot left activates both motors in opp. directions to turn left.
 */
void pivotLeft() 
{
  rightForward(fast_R);
  leftForward(fast_L);
  delay(600);
  robotStop();
  rightForward(fast_R);
  leftForward(back_fast_L);
  delay(200);

  updateCompass(0);

  int ready = 0;
  while (ready != sampler2)
  {
    ready += checkLine();
  }
  intersectFlag = 0;
}

/*
 * Description: Pivot right activates both motors in opp. directions to turn right.
 */
void pivotRight() 
{
  rightForward(fast_R);
  leftForward(fast_L);
  delay(600);
  robotStop();
  leftForward(fast_L);
  rightForward(back_fast_R);
  delay(200);
  
  updateCompass(1);

  int ready = 0;
  while (ready != sampler2)
  {
    ready += checkLine();
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

  int ready = 0;
  while (ready != sampler2)
  {
    ready += checkLine();
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

  int ready = 0;
  while (ready != sampler2)
  {
    ready += checkLine();
  }
  delay(50);
  intersectFlag = 0;
}


void pivot180(){
    leftForward(fast_L);
    rightForward(back_fast_R);
    delay(1000);

    updateCompass(1);
    updateCompass(1);

    int ready = 0;
    while (ready != sampler2)
    {
      ready += checkLine();
    }
    intersectFlag=0;
}

/* -------------------------------
 *         LINE FOLLOWER 
 * -------------------------------
 */

void pd_followLine()
{
  //working slow settings:
  // 0.9, 85 +/- 5

  int KD = 0.9;
  int tempspeed;

  checkLineSensors();
  // right drifts onto white
  if (LSR < 500)
  {
    int rdiff = (850 - LSL) / 850;
    int tempspeed = 85 + rdiff * KD * 5;
    right.write(tempspeed);
  }
  else if (LSL < 500)
  {
    int ldiff = (800 - LSL) / 800;
    int tempspeed = 95 - ldiff * KD * 5;
    left.write(tempspeed);
  }
  else
  {
    leftForward(fast_L);
    rightForward(fast_R);
  }
}

/* Check whether the robot is at an intersect; 
   set intersectFlag = 1 if true */
void checkIntersect()
{
  checkLineSensors();
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

  for (int x = 0; x < 8; x++)
  {
    av_L += analogRead(lineSensorL);
    av_M += analogRead(lineSensorM);
    av_R += analogRead(lineSensorR);
  }

  // LSL = av_L / 6;
  // LSM = av_M / 6;
  // LSR = av_R / 6;
  LSL = av_L >> 3;
  LSM = av_M >> 3;
  LSR = av_R >> 3;
  // sprintf(buffer,"LSL: %d, LSM: %d, LSR: %d", LSL, LSM, LSR);
  // Serial.println(buffer);

}

/* Get the readings from distance sensors (for wall detection)*/
void checkDistSensors() {
  int sample = sampler;

  int av_Dist_L = 0;
  int av_Dist_R = 0;
  int av_Dist_F = 0;

  for (int x = 0; x < sample; x++)
  {
    // SWEEP PINS:
    // FRONT MUX = 2
    digitalWrite(mux_a, LOW);       
    digitalWrite(mux_b, HIGH);       
    digitalWrite(mux_c, LOW);   
    av_Dist_F +=  analogRead(A3);

    // Left MUX =  0
    digitalWrite(mux_a, LOW);       
    digitalWrite(mux_b, LOW);       
    digitalWrite(mux_c, LOW);   
    av_Dist_L += analogRead(A3);
    
    // RIGHT MUX = 1
    digitalWrite(mux_a, HIGH);       
    digitalWrite(mux_b, LOW);       
    digitalWrite(mux_c, LOW);   
    av_Dist_R += analogRead(A3);

  }

  distLeft = av_Dist_L / sample;
  distFront = av_Dist_F / sample;
  distRight = av_Dist_R / sample;
  
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
    // sprintf(buffer, "Front: %d", distFront);
    Serial.println(buffer);
  }
  //right sensor detects wall wall value with shift by compass to reorient
  if (distRight<distHigh && distRight>distLow){
    switch (compass){
      case 0: 
        walls |= (0b0100); //wall is east
        break;
      case 1: 
        walls |= (0b0010); //wall is south
        break;
      case 2: 
        walls |= (0b0001); //wall is west
        break;
      case 3: 
        walls |= (0b1000); //wall is north
        break;
      }
      // sprintf(buffer, "Right: %d", distFront);
      Serial.println(buffer);   
  }
  //left sensor detects wall, set wall value with shift by compass to reorient
  if (distLeft<distHigh && distLeft>distLow){
    switch (compass){
      case 0: 
        walls |= (0b0001); //set wall on left when facing north
        break;
      case 1: 
        walls |= (0b1000); //wall is north
        break;
      case 2: 
        walls |= (0b0100); //wall is east
        break;
      case 3: 
        walls |= (0b0010); //wall is south
        break;
    }
    // sprintf(buffer, "Left: %d", distFront);
    Serial.println(buffer);
  }
  sprintf(buffer,"CheckWall Walls: %d, Compass: %d", walls, compass);
  Serial.println(buffer);
}

/*Simple code for avoiding walls while moving in the maze
May be modified to include robot avoidance*/
void checkWallOld() {
  checkDistSensors();
  //wall on right, no wall in front
  if ((distRight < distHigh && distRight > distLow) && 
    (distFront>distHigh || distFront<distLow)) { 
    //go straight
    // Serial.println("Forward");
    digitalWrite(ledPin_right, HIGH);
    pd_followLine();
    digitalWrite(ledPin_right, LOW);
  }
  //wall on right and in front, no wall on left
  else if ((distRight < distHigh && distRight > distLow) && (distFront<distHigh && distFront>distLow) && (distLeft>distHigh || distLeft<distLow)) {
    //rotate left 90
    digitalWrite(ledPin_front, HIGH);
    digitalWrite(ledPin_right, HIGH);
    // turnLeft();
    pivotLeft();
    // Serial.println("Turn Left");
    digitalWrite(ledPin_front, LOW);
    digitalWrite(ledPin_right, LOW);
  }
  //no wall on right and wall in front
  else if ((distRight > distHigh || distRight < distLow) && distFront<distHigh && distFront>distLow){
    //take a right turn
    digitalWrite(ledPin_front, HIGH);
    // turnRight();
    pivotRight();
    // Serial.println("Turn Right");
    digitalWrite(ledPin_front, LOW);
  }
  //wall on left, right and in front
  else if ((distRight < distHigh && distRight > distLow) && (distFront<distHigh && distFront>distLow) && (distLeft<distHigh && distLeft>distLow)) {
    //dead end
    digitalWrite(ledPin_front, HIGH);
    digitalWrite(ledPin_right, HIGH);
    pivotRight();
    pivotRight();
    // Serial.println("Pivot 180");
    digitalWrite(ledPin_front, LOW);
    digitalWrite(ledPin_right, LOW);
  }
  // no wall on right, no wall in front
  else{
    //go forward
    pd_followLine();
    // Serial.println("Else Forward");

  }
}
