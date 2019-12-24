//#include <Maze_DFS.c>
//#include <Robot_Move.c>
//#include <Servo.h>

//#include "stdio.h"
//#include "stdlib.h"
#include <Servo.h>

typedef uint8_t byte;

char buffer[60];
int start = 0;

/**********************************************/
/*                   MAZE DFS                 */
/**********************************************/

//orientation of bot (0-3 = N-W)
enum Compass{
  North,
  East,
  South,
  West
};

typedef struct {
  byte vertex;
  byte x;
  byte y;
  byte walls; //write to walls if unvisited node
  byte adjNodes[4]; //nodes vertex values. 0-3 <=> N-W
  byte back;
  // struct node *next
} node;

byte compass=North; //default north to start, bottom left corner
byte direction;
byte backDirection = South; //direction behind bot;
byte diff = 0; //difference between compass and direction
byte currentVertex = 0; //vertex 1-100
byte prevVertex = 0;
//default to (0,0), bottom left corner
byte xpos, ypos = 0;

node *G[100];

bool wallLeft = 0;
bool wallRight = 0;
bool WallForward =0;

byte walls;

/************** MAZE DFS - END ****************/


/* -------------------------------
 *           THRESHOLDS
 * -------------------------------
 */
//servo speeds
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

// line follower thresholds
#define left_to 500
#define mid_to 500
#define right_to 500

// dist sensors thresholds
#define distLow 200 //far
#define distHigh 650 //close
#define frontDist 235

// turning thresholds
#define pivot_overshoot 250
#define turn_check_overshoot 200
#define check_line_delay 20

int sampler = 5;
int sampler2 = 5;

/* -------------------------------
 *        PIN DEFINITIONS
 * -------------------------------
 */
 
Servo left;
Servo right;
byte pinLeft = 5;
byte pinRight = 6;

// line sensors
int lineSensorL = A0;
int lineSensorM = A1;
int lineSensorR = A2;
int LSL;
int LSM;
int LSR;
bool intersectFlag = 0;

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


////////////////////////////////////////////////
/*----------------- MAZE DFS -----------------*/
////////////////////////////////////////////////

//array containing values for adjacent verticies based on vertex numbers
// int adj[100][4]; 
//TODO: figure out out to create walls on the perimeter?
//  probably can't init just NULL for everything, so what?
/* Setup for DFS */
void DFSInit(){
    for(int i=1;i<100;i++){
      G[i]=NULL;
    }
}

void DFSInitFirstNode(){
  node *firstNode;
  firstNode=(node*)malloc(sizeof(node));
  firstNode->vertex = 0;
  firstNode->x = 0;
  firstNode->y = 0;
  checkWall();
  firstNode->walls = walls;
  addAllEdges(firstNode, North, prevVertex); //prevVertex should be 0
  G[0]=firstNode;
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


//TODO: REWRITE to deal with node behind as opposed absolute directions. Compensate with current compass direction 

/* Adds Vertex values of all adjacent nodes to current node.
Does not create new nodes. Adds back pointer to currentNode as well.
-1 written if there is a wall present
-2 prewritten if this is an uninitialized node position
*/

//potential modifcation: instead of adding int vertex to adjNodes,
//could do pointer to from nodes from graph
void addAllEdges(node *currNode, byte direction, int prevNode){
  Serial.println("addAllEdges");
  checkWall();
  currNode->walls = walls;
  Serial.println("Walls: ");
  Serial.println(walls);
  //update backDirection
  backDirection = (direction+2)%4;
  currNode->back = backDirection;
  //no wall north
  if(currNode->adjNodes[0]==-2){ //need something for initial perimeter walls
    Serial.println("Check for path north");
    currNode->adjNodes[0] = (!(walls & 0b1000)) ? (currentVertex + 10) : -1;
    if (!(walls & 0b1000)) Serial.println("Yes Path north");
  }

  //no wall east
  if(currNode->adjNodes[1]==-2){
    Serial.println("Check for path east");
    currNode->adjNodes[1] = (!(walls & 0b0100)) ? (currentVertex + 1) : -1;
    if (!(walls & 0b0100)) Serial.println("Yes Path east");
  }
  
  //no wall south
  if(currNode->adjNodes[2]==-2){
      Serial.println("Check for path south");
    currNode->adjNodes[2] = (!(walls & 0b0010)) ? (currentVertex - 10) : -1;
    if (!(walls & 0b0010)) Serial.println("Yes Path south");
  }

  //no wall west
  if(currNode->adjNodes[3]==-2){  
    Serial.println("Check for path west");
    currNode->adjNodes[3] = (!(walls & 0b0001)) ? (currentVertex - 1) : -1;
    if (!(walls & 0b0001)) Serial.println("Yes Path west");
  }
  //update wall position at for the back direction to account for error of not having a back sensor
  if (currNode->adjNodes[backDirection] == -2){
    currNode->adjNodes[backDirection] = prevNode;
  }
}

/* Changes compass to get to new edge using current compass and new direction. 
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
  else if (compass == (direction+1)%4) {
    Serial.println("Turn: left");
    pivotLeft();
    pd_followLine();
  }
  else if ((compass+1)%4 == direction) {
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

  //update global position based on any compass updates done with movement
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
  updateVertex();
  while(!intersectFlag){
    pd_followLine();
    checkIntersect();
    // sprintf(buffer,"moveToNearbyNode: Intersect: %d", intersectFlag);
    // Serial.println(buffer);
    // Serial.print("moveToNearbyNode: Intersect:");
    // Serial.print(intersectFlag);
    // Serial.println("");
  }
}

/* TODO: Outputs instructions for return path across visited nodes,
Bubble Up for DFS. */
void backTrack(int backDirection){
  Serial.println("backTrack");
  moveToNearbyNode(backDirection);
  bool newNode = 0;
  updateVertex();
  //check through adjNodes of currNode for a newNode
  for (int i=0; i<4; i++){
    if (G[G[currentVertex]->adjNodes[i]]==NULL){
      newNode = 1;
      moveToNearbyNode(i); //move to newNode based on direction
      DFS(currentVertex); //continue DFS
    } 
  }
  if (!newNode){
    backTrack(G[currentVertex]->back);
  }
}


/* Depth First Search traversal on a starting point. 
DFS should be called at an intersection 
Vertex = global position, had byte direction value but can't remember why */
void DFS(int Vertex)
{
  //printf("\n%d",i);
  // delay(1000);
  // Serial.println("Delay 1s");

  //DFS called upon new node, must move to node
  // robot do things

  //currently visiting a new node
  //initilize the node
  updateVertex();
  node *currNode;
  // currNode=(node*)malloc(sizeof(node));
  currNode->vertex = currentVertex;
  currNode->x = xpos;
  currNode->y = ypos;
  currNode->adjNodes[0] = -2;
  currNode->adjNodes[1] = -2;
  currNode->adjNodes[2] = -2;
  currNode->adjNodes[3] = -2;

  // checkWall();
  // currNode->walls = walls;
//  currNode->adjNodes[]={-2, -2, -2, -2};

  //create new edges for a node, adds backDirection using direction
  addAllEdges(currNode, direction, prevVertex);

  G[Vertex] = currNode;

  prevVertex = currentVertex;
  // Base case, while the adjacent neighbor of the current node is not NULL
  // check for available directions starting from north=0
  direction = North;
  while(direction<4){
    //Vertex = currNode->vertex;
    sprintf(buffer,"Direction: %d", direction);
    Serial.println(buffer);
    
    if(intersectFlag) {
      // robotStop(); //pause because things are happening faster than I can mentally comprehend
      sprintf(buffer,"DFS Still Intersect: %d", intersectFlag);
      Serial.println(buffer);
      //idiot, you haven't visited anything yet   
      //choose new node to move to
      // delay(1000);
      // Serial.println("Delay 1s");

      
      // check for valid direction and if node in that direction is still unvisited (NULL)
      // moves to first possible direction starting from north
      if ((G[Vertex]->adjNodes[direction]==-2 || G[Vertex]->adjNodes[direction]>-1) && (G[G[Vertex]->adjNodes[direction]]==NULL)){
      // if ((G[Vertex]->adjNodes[direction]>-2) || (G[G[Vertex]->adjNodes[direction]]==NULL)){
        sprintf(buffer,"Direction: %d is NULL", direction);
        Serial.println(buffer);
        moveToNearbyNode(direction); // move to nearby node based on null direction
        // robot will have moved to a new node
        // Call DFS again assuming that we have made it to the new node.
        // DFS on new vertex, how does the starting direction affect our algorithm?
        DFS(currentVertex);
      }
      direction++;
    }
  }

  //backtrack to currNode first??
  //need return path damn...
  //calculate shortest path?  
  //visiting previous nodes now, not making new nodes 
  backTrack(currNode->back);
  
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
  rightForward(med_R);
  leftForward(med_L);
  delay(300);
  robotStop();
  rightForward(med_R);
  leftForward(back_med_L);
  delay(300);

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
  rightForward(med_R);
  leftForward(med_L);
  delay(300);
  robotStop();
  leftForward(med_L);
  rightForward(back_med_R);
  delay(300);
  
  updateCompass(1);

  int ready = 0;
  while (ready != sampler2)
  {
    ready += checkLine();
  }
  intersectFlag = 0;
}

// void turnLeft()
// {
//   delay(250);
//   robotStop();
//   rightForward(fast_R);
//   leftForward(back_slow_L);
//   delay(200);

//   int ready = 0;
//   while (ready != sampler2)
//   {
//     ready += checkLine();
//   }
//   intersectFlag = 0;
// }


// void turnRight()
// {
//   delay(250);
//   robotStop();
//   leftForward(fast_L);
//   rightForward(back_slow_R);
//   delay(200);

//   int ready = 0;
//   while (ready != sampler2)
//   {
//     ready += checkLine();
//   }
//   delay(50);
//   intersectFlag = 0;
// }


void pivot180(){
    leftForward(med_fast_L);
    rightForward(back_med_fast_R);
    delay(500);
    leftForward(slow_L);
    rightForward(back_slow_R);

    updateCompass(1);
    updateCompass(1);

    int ready = 0;
    while (ready != 3)
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
  //default to no surrounding walls
  wallLeft = 0;
  wallRight = 0;
  WallForward =0;

  //front sensor detects wall, set wall value with shift by compass to reorient
  if (distFront>frontDist){
    WallForward=1;
    walls |= (0b1000>>compass); //set wall in front when facing north
    // // sprintf(buffer, "Front: %d", distFront);
    // Serial.println(buffer);
  }
  //right sensor detects wall wall value with shift by compass to reorient
  if (distRight<distHigh && distRight>distLow){
    wallRight=1;
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
    wallLeft=1;
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
  sprintf(buffer,"CheckWall WallFront: %d, WallLeft: %d, WallRight: %d, Compass: %d", WallForward, wallLeft, wallRight, compass);
  Serial.println(buffer);
}

/* DOESN'T WORK: Simple code for avoiding walls while moving in the maze
May be modified to include robot avoidance.*/
void checkWallOld() {
  checkDistSensors();
  //wall on right, no wall in front
  if ((distRight < distHigh && distRight > distLow) && 
    (distFront<frontDist)) { 
    //go straight
    // Serial.println("Forward");
    digitalWrite(ledPin_right, HIGH);
    pd_followLine();
    digitalWrite(ledPin_right, LOW);
  }
  //wall on right and in front, no wall on left
  else if ((distRight < distHigh && distRight > distLow) && (distFront>frontDist) && (distLeft>distHigh || distLeft<distLow)) {
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
  else if ((distRight > distHigh || distRight < distLow) && (distFront>frontDist)){
    //take a right turn
    digitalWrite(ledPin_front, HIGH);
    // turnRight();
    pivotRight();
    // Serial.println("Turn Right");
    digitalWrite(ledPin_front, LOW);
  }
  //wall on left, right and in front
  else if ((distRight < distHigh && distRight > distLow) && (distFront>frontDist) && (distLeft<distHigh && distLeft>distLow)) {
    //dead end
    digitalWrite(ledPin_front, HIGH);
    digitalWrite(ledPin_right, HIGH);
    pivot180();
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

/* Check if there are walls around the robot; make it react accordingly */
void rightHandFollow() {
  
  checkDistSensors();
    // all walls
  if((distRight<distHigh && distRight>distLow) && (distFront>235) && (distLeft<distHigh && distLeft>distLow)){
     pivot180();
  }
    //wall on right and in front
  else if ((distRight < distHigh && distRight > distLow) && (distFront>235)) {
    //rotate left 90
    digitalWrite(ledPin_front, HIGH);
    digitalWrite(ledPin_right, HIGH);
    pivotLeft();
    digitalWrite(ledPin_front, LOW);
    digitalWrite(ledPin_right, LOW);
  }
  else if ((distRight < distHigh && distRight > distLow) && (distFront<235)) { 
    //go straight
    digitalWrite(ledPin_right, HIGH);
    pd_followLine();
    digitalWrite(ledPin_right, LOW);
  }

  //no wall on right and wall in front
  else if ((distRight > distHigh || distRight < distLow) && (distFront>235)){
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


/* -------------------------------
 *        MAIN INTEGRATION
 * -------------------------------
 */

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
  //push button start
  if (digitalRead(13) == 1){
    start ^= 1;
    delay(250);
    sprintf(buffer,"start: %d", start);
    Serial.println(buffer);
  }
//   Right Hand Following
//  if (start){
//   while(1) {
//      checkIntersect();
//      pd_followLine();
//      if(intersectFlag) {
//        rightHandFollow();
//        intersectFlag = 0;
//      }
//    }
//  }

  /*must orient robot north, bottom left corner

    0 0 0 0 0 0 0 0 0
    0 0 0 0 0 0 0 0 0
    0 0 0 0 0 0 0 0 0
    0 0 0 0 0 0 0 0 0
    0 0 0 0 0 0 0 0 0
    0 0 0 0 0 0 0 0 0
    0 0 0 0 0 0 0 0 0
  ^ 0 0 0 0 0 0 0 0 0
  | R 0 0 0 0 0 0 0 0
  */
  //Here we have already initilized DFS maze. We wish to start with our robot over the first intersection
   if (start){
       //after start signal, robot should add all surrounding walls to first node.
       DFSInitFirstNode(); 
       if (walls & 0b1000){ //wall in front at start
         moveToNearbyNode(East);//probably need to modify for a good pivot right?
       }
       else{
         moveToNearbyNode(North);
       }
       //should be at 2nd node now, at the intersect
       if(intersectFlag) {
         DFS(1);
         intersectFlag = 0;
       }
     }

}
