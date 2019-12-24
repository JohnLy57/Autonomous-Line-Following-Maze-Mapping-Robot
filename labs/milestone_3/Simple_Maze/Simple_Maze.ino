
#include <Servo.h>

typedef uint8_t byte;

//char buffer[60];
int start = 0;


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
#define frontDist 150

// turning thresholds
#define pivot_overshoot 250
#define turn_check_overshoot 200
#define check_line_delay 20

byte sampler = 5;
byte sampler2 = 5;

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

int travelCounter;
int nextDirection;

int m = 0;


/**************    MAZE VAR    ****************/


//orientation of bot (0-3 = N-W)
enum Compass{
  North,
  East,
  South,
  West
};

typedef struct{
    byte direction;
    byte recentdir;
    bool north;
    bool west;
    bool east;
    bool south;
    bool visited;
}node;

node init_node= {0,0,0,0,0,0,0};

node Maze[9][9] = {
    {init_node, init_node, init_node, init_node, init_node, init_node, init_node, init_node, init_node},
    {init_node, init_node, init_node, init_node, init_node, init_node, init_node, init_node, init_node},
    {init_node, init_node, init_node, init_node, init_node, init_node, init_node, init_node, init_node},
    {init_node, init_node, init_node, init_node, init_node, init_node, init_node, init_node, init_node},
    {init_node, init_node, init_node, init_node, init_node, init_node, init_node, init_node, init_node},
    {init_node, init_node, init_node, init_node, init_node, init_node, init_node, init_node, init_node},
    {init_node, init_node, init_node, init_node, init_node, init_node, init_node, init_node, init_node},
    {init_node, init_node, init_node, init_node, init_node, init_node, init_node, init_node, init_node},
    {init_node, init_node, init_node, init_node, init_node, init_node, init_node, init_node, init_node},
};

//debug structure:
typedef struct{
  byte x;
  byte y;
  byte taken;
  bool north;
  bool west;
  bool east;
  bool south;
  bool backtrack;
  bool visited;
}printnode;

printnode init_printnode = {0,0,0,0,0,0,0,0,0};
printnode printMaze [36] = {
    init_printnode, init_printnode, init_printnode, init_printnode, init_printnode, init_printnode, init_printnode, init_printnode, init_printnode,
    init_printnode, init_printnode, init_printnode, init_printnode, init_printnode, init_printnode, init_printnode, init_printnode, init_printnode,
    init_printnode, init_printnode, init_printnode, init_printnode, init_printnode, init_printnode, init_printnode, init_printnode, init_printnode,
    init_printnode, init_printnode, init_printnode, init_printnode, init_printnode, init_printnode, init_printnode, init_printnode, init_printnode};
 

byte direction;
byte compass = 0;

byte xpos, ypos = 0;

bool wallLeft = 0;
bool wallRight = 0;
bool wallForward = 0;

bool walls[4] = {0, 0, 0, 0} ;

// node currentNode;


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

//  updateCompass(0);

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
  
//  updateCompass(1);

  int ready = 0;
  while (ready != sampler2)
  {
    ready += checkLine();
  }
  intersectFlag = 0;
}

void pivot180(){
    leftForward(med_fast_L);//med_fast_L
    rightForward(back_med_fast_R);
    delay(500);
    leftForward(slow_L);
    rightForward(back_slow_R);

//    updateCompass(1);
//    updateCompass(1);

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
// WHITE IS LOW, BLACK IS HIGH

void pd_followLine()
{
//      int l_light = analogRead(lineSensorL);
//    int m_light = analogRead(lineSensorM);
//    int r_light =analogRead(lineSensorR);
//    int m_thresh = ((r_light + l_light)/4) + 50;
////    Serial.print("Left: "); Serial.println(l_light);
////    Serial.print("Middle: "); Serial.println(m_light);
////    Serial.print("Right: "); Serial.println(r_light);
//    if (l_light > 650 && m_light > m_thresh) {
//      left.write(med_L);
//      right.write(slow_R);
////      Serial.println("veered left");
////      Serial.print("Left: "); Serial.println(l_light);
////      Serial.print("Middle: "); Serial.println(m_light);
////      Serial.print("Right: "); Serial.println(r_light);
//    } else if (r_light > 650 && m_light > m_thresh) {
//      left.write(slow_L);
//      right.write(med_R);
////      Serial.println("veered right");
////      Serial.print("Left: "); Serial.println(l_light);
////      Serial.print("Middle: "); Serial.println(m_light);
////      Serial.print("Right: "); Serial.println(r_light);
//    } else {
//      left.write(93);
//      right.write(89);
////      Serial.println("on line");
////      Serial.print("Left: "); Serial.println(l_light);
////      Serial.print("Middle: "); Serial.println(m_light);
////      Serial.print("Right: "); Serial.println(r_light);
//    }

    //START PROBLEMS
//  //working slow settings:
//  // 0.9, 85 +/- 5
//
  int KD = 0.9;
  int tempspeed;

  checkLineSensors();
  // right drifts onto white
  if (LSR < 400)
  {
    int rdiff = (850 - LSR) / 850;
    int tempspeed = 87 + rdiff * KD * 3;
    right.write(tempspeed);
  }
  else if (LSL < 400)
  {
    int ldiff = (850 - LSL) / 850;
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

  for (int x = 0; x < 4; x++)
  {
    av_L += analogRead(lineSensorL);
    av_M += analogRead(lineSensorM);
    av_R += analogRead(lineSensorR);
  }

  // LSL = av_L / 6;
  // LSM = av_M / 6;
  // LSR = av_R / 6;
  LSL = av_L >> 2;
  LSM = av_M >> 2;
  LSR = av_R >> 2;
  
//    Serial.println("---------");
//    Serial.println(LSL);
//    Serial.println(LSM);
//    Serial.println(LSR);
  // sprintf(buffer,"LSL: %d, LSM: %d, LSR: %d", LSL, LSM, LSR);
  // Serial.println(buffer);

}

/* Get the readings from distance sensors (for wall detection)*/
void checkDistSensors() {
  int sample = 5;

  int av_Dist_L = 0;
  int av_Dist_R = 0;
  int av_Dist_F = 0;

  for (int x = 0; x < sample; x++)
  {
    // SWEEP PINS:
    // FRONT MUX = 2
    // digitalWrite(mux_a, LOW);       
    // digitalWrite(mux_b, HIGH);       
    // digitalWrite(mux_c, LOW);   
    av_Dist_F +=  analogRead(A5);

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

  //Serial.println(distFront);
  
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
//  walls = 0;
  //default to no surrounding walls
  wallLeft = 0;
  wallRight = 0;
  wallForward =0;

  //front sensor detects wall, set wall value with shift by compass to reorient
  if (distFront>frontDist){
    wallForward=1;
 
    // walls[0];
  }
  //right sensor detects wall wall value with shift by compass to reorient
  if (distRight<distHigh && distRight>distLow){
    wallRight=1;
      
      // sprintf(buffer, "Right: %d", distFront);
    //   Serial.println(buffer);   
}
  //left sensor detects wall, set wall value with shift by compass to reorient
  if (distLeft<distHigh && distLeft>distLow){
    wallLeft=1;

    // sprintf(buffer, "Left: %d", distFront);
    
  }
//  sprintf(buffer,"Front: %d, Left: %d, Right: %d", WallForward, wallLeft, wallRight);
//  Serial.println(buffer);

//    Serial.print("Left: ");
//    Serial.println(wallLeft);
//    Serial.print("Front: ");
//    Serial.println(wallForward);
//    Serial.print("Right: ");
//    Serial.println(wallRight);
}

/* Check if there are walls around the robot; make it react accordingly */
void rightHandFollow() {
  checkDistSensors();
    // all walls
  if((distRight<distHigh && distRight>distLow) && (distFront>frontDist) && (distLeft<distHigh && distLeft>distLow)){
     pivot180();
  }
    //wall on right and in front
  else if ((distRight < distHigh && distRight > distLow) && (distFront>frontDist)) {
    //rotate left 90
    digitalWrite(ledPin_front, HIGH);
    digitalWrite(ledPin_right, HIGH);
    pivotLeft();
    digitalWrite(ledPin_front, LOW);
    digitalWrite(ledPin_right, LOW);
  }
  else if ((distRight < distHigh && distRight > distLow) && (distFront<frontDist)) { 
    //go straight
    digitalWrite(ledPin_right, HIGH);
    pd_followLine();
    digitalWrite(ledPin_right, LOW);
  }

  //no wall on right and wall in front
  else if ((distRight > distHigh || distRight < distLow) && (distFront>frontDist)){
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



////////////////////////////////////////////////
/*----------------- MAZE  -----------------*/
////////////////////////////////////////////////

// Called after wallcheck and determining unvisited node.
void moveToNearbyNode(int direction){
    
    //travelCounter++;
    intersectFlag = 0;    
    // Turn
    int diff = abs(direction - Maze[xpos][ypos].recentdir); // previously direction - Maze[xpos][ypos].direction

    if (diff == 0){
        //If shit goes wrong come back here!!!!
        //Serial.println("forward");
        
        int counter = 0;
        while (counter < 50){
            pd_followLine();
            counter +=1;
        }

    }
    else if (diff == 2){
       // Serial.println("back");
        pivot180();
        pd_followLine();
    }
    else if (Maze[xpos][ypos].recentdir == (direction+1)%4){
      //  Serial.println("left");
        pivotLeft();
        pd_followLine();
    }
    else if ((Maze[xpos][ypos].recentdir+1)%4 == direction){
      //  Serial.println("right");
        pivotRight();
        pd_followLine();
    }
    //Serial.print("Direction:");
    //Serial.println(direction);
    switch(direction){
        case North: 
          ypos++; 
          Maze[xpos][ypos].south = 1; 
          break;
        case East: 
          xpos++;
          Maze[xpos][ypos].west = 1; 
          break;
        case West: 
          xpos--;
          Maze[xpos][ypos].east = 1;
          break;
        case South: 
          ypos--;
          Maze[xpos][ypos].north = 1;
          break;
    }
    
   
    if (!Maze[xpos][ypos].visited) Maze[xpos][ypos].direction = direction;
    Maze[xpos][ypos].recentdir = direction;
    
    //Serial.print("Direction:");
    //Serial.println(Maze[xpos][ypos].direction);


    // Move to next intersection  
    while(!intersectFlag){
      pd_followLine();
      checkIntersect();
    }
//    Serial.print("****************");
//    Serial.print("(");
//    Serial.print(xpos);
//    Serial.print(",");
//    Serial.print(ypos);
//    Serial.println(")");
    
    left.write(91);
    right.write(89);

}

// Decision making method.
int directionPick(){    
    int nextMove = 0;
    checkWall();
    // Check sensors and see what's on the market.
    if(!Maze[xpos][ypos].visited){
        switch(Maze[xpos][ypos].direction)
        {
            case North: 
                Maze[xpos][ypos].east = wallRight;
                Maze[xpos][ypos].west = wallLeft;
                Maze[xpos][ypos].north = wallForward;
                Maze[xpos][ypos].south = 1;
            break;
            case East: 
                Maze[xpos][ypos].east = wallForward;
                Maze[xpos][ypos].north = wallLeft;
                Maze[xpos][ypos].south = wallRight;
                Maze[xpos][ypos].west = 1;

            break;
            case West: 
                Maze[xpos][ypos].east = 1;
                Maze[xpos][ypos].west = wallForward;
                Maze[xpos][ypos].north = wallRight;
                Maze[xpos][ypos].south = wallLeft;
            break;
            case South: 
                Maze[xpos][ypos].east = wallLeft;
                Maze[xpos][ypos].west = wallRight;
                Maze[xpos][ypos].south = wallForward;
                Maze[xpos][ypos].north = 1;

            break;
        }
    }
//
//    switch(Maze[xpos][ypos].recentdir)
//    {
//            case North: 
//                Maze[xpos][ypos].east |= wallRight;
//                Maze[xpos][ypos].west |= wallLeft;
//                Maze[xpos][ypos].north |= wallForward;
//            break;
//            case East: 
//                Maze[xpos][ypos].east |= wallForward;
//                Maze[xpos][ypos].north |= wallLeft;
//                Maze[xpos][ypos].south |= wallRight;
//            break;
//            case West: 
//                Maze[xpos][ypos].west |= wallForward;
//                Maze[xpos][ypos].north |= wallRight;
//                Maze[xpos][ypos].south |= wallLeft;
//            break;
//            case South: 
//                Maze[xpos][ypos].east |= wallLeft;
//                Maze[xpos][ypos].west |= wallRight;
//                Maze[xpos][ypos].south |= wallForward;
//            break;
//    }
//    
    // Checks nearby node for paths to take. Prevents useless looping.
    if (xpos-1 >=0){
      if (Maze[xpos-1][ypos].east) Maze[xpos][ypos].west =1;
    }
    if (xpos+1 <= 8){
      if (Maze[xpos+1][ypos].west) Maze[xpos][ypos].east =1;
    }
    if (ypos+1 <= 8){
      if (Maze[xpos][ypos+1].south) Maze[xpos][ypos].north =1;
    }
        if (ypos-1 >= 0){
      if (Maze[xpos][ypos-1].north) Maze[xpos][ypos].south =1;
    }

    // Pick next direction based on available.
//    Serial.print("Node was previously visited:");
//    Serial.print(Maze[xpos][ypos].visited);
    //NESW
//    Serial.print(" N:");
//    Serial.print(Maze[xpos][ypos].north);
//    Serial.print(" E:");
//    Serial.print(Maze[xpos][ypos].east);
//    Serial.print(" S:");
//    Serial.print(Maze[xpos][ypos].south);
//    Serial.print(" W:");
//    Serial.println(Maze[xpos][ypos].west);
    
    //FIRST SET OF PRINTMAZE
//    printMaze[travelCounter].x = xpos;
//    printMaze[travelCounter].y = ypos;
//    printMaze[travelCounter].y = Maze[xpos][ypos].visited;
//    printMaze[travelCounter].north = Maze[xpos][ypos].north;
//    printMaze[travelCounter].west = Maze[xpos][ypos].west;
//    printMaze[travelCounter].east = Maze[xpos][ypos].east;
//    printMaze[travelCounter].south = Maze[xpos][ypos].south;

    Maze[xpos][ypos].visited = 1;

//    Serial.print("Node is now visited:");
//    Serial.println(Maze[xpos][ypos].visited);

    if(Maze[xpos][ypos].north==0){
        nextMove = North;
        Maze[xpos][ypos].north=1;
    }
    else if(Maze[xpos][ypos].east==0){
        nextMove = East;
        Maze[xpos][ypos].east=1;
    }
    else if(Maze[xpos][ypos].west==0){
        nextMove = West;
        Maze[xpos][ypos].west =1;
    }
    else if(Maze[xpos][ypos].south==0){
        nextMove = South;
        Maze[xpos][ypos].south =1;
    }
    else{
        //Serial.println("backTrack");
        printMaze[travelCounter].backtrack = 1;
        nextMove = (Maze[xpos][ypos].direction + 2)%4;
    }

    //SECOND SET OF PRINTMAZE
//    printMaze[travelCounter].taken = nextMove;

    return nextMove;
}


/* -------------------------------
 *        MAIN INTEGRATION
 * -------------------------------
 */

void setup(){

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

  // currentNode = Maze[xpos][ypos];
    Maze[0][0].south=1;
}
 

void loop(){
if (digitalRead(13) == 1){
    start = 1;
//    delay(250);
//    sprintf(buffer,"start: %d", start);
//    Serial.println(start);
   left.write(91);
    right.write(89);
  }
//while(1){  
//  int n = analogRead(A5);
//  Serial.println(n);
//}
if(start){
//  while(1){
//    checkIntersect();
//    pd_followLine();
//    if(intersectFlag){
//      rightHandFollow();
//      intersectFlag=0;
//    }
//  }
    while(1){
      nextDirection = 0;
      nextDirection = directionPick();
      moveToNearbyNode(nextDirection);
    }
    //if (Serial.available() > 0){
    //  robotStop();
    //}

//  rightHandFollow();
  }
}
