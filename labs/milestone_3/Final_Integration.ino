#include <Servo.h>

typedef uint8_t byte;

int start = 0;

/* -------------------------------
 *           THRESHOLDS
 * -------------------------------
 */
// Servo Speeds
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

// Line Following Thresholds
#define left_to 378     
#define mid_to 430
#define right_to 432

#define post_movement_delay 50

// Distance Sensor Thresholds
#define sideDist 180    // Above is wall.
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
int lineSensorL = A3;
int lineSensorM = A5;
int lineSensorR = A4;
long l_light;
long m_light;
long r_light;
int m_thresh;
bool intersectFlag = 0;

// dist sensors
int DistSensorL = A0;
int DistSensorF = A2;
int DistSensorR = A1;
int distFront = 0;
int distLeft = 0;
int distRight = 0;


// BELOW THIS  IT IS NOT USED
// Large print-out.
int travelCounter;

int m = 0;
// ------------------------------

/* -------------------------------
 *        FXN DEFINITIONS
 * -------------------------------
 */
 int tempspeed;

// Averaging variables for wall sensors.
int av_Dist_L = 0;
int av_Dist_R = 0;
int av_Dist_F = 0;

/* -------------------------------
 *        MAZE VARIABLES
 * -------------------------------
 */

//orientation of bot (0-3 = N-W)
enum Compass{
  North,
  East,
  South,
  West
};

typedef struct{
    byte direction; //should change naming to backPointer
//    byte recentdir;
    bool north;
    bool west;
    bool east;
    bool south;
    bool visited;
}node;

node init_node= {0,0,0,0,0,0};

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

byte xpos, ypos = 0;

// byte direction;
byte nextDirection = 0; //global var, passed into moveToNearbyNode
byte recentDirection = 0; //compass variable
byte diff;

bool wallLeft = 0;
bool wallRight = 0;
bool wallForward = 0;


/* -------------------------------
 *         BASIC MOVEMENT 
 * -------------------------------
 */

//NEW PIVOT FUNCTIONS:
//Robot will drive forward, begin a turn.
//Until it hits the middle, then stop.
//Then delay slightly to bring us onto track.
//DO NOT ALTER THE DELAYS.
//CHECK THRESHOLDS FIRST.

void pivotLeft() 
{
  l_light=0;
  m_light=0;
  r_light=0;

  right.write(slow_R);
  left.write(slow_L);
  delay(500);
  
  //STOP
  right.write(90);
  left.write(90);
  right.write(slow_R);
  left.write(back_slow_L);
  delay(300);

  // DO AN INITIAL READ
  for(int x =0; x<4; x++){
    m_light += analogRead(lineSensorM);
    l_light += analogRead(lineSensorL);
  }
  m_light = m_light/4;
  l_light = l_light/4;
  
  while (!(m_light < mid_to && l_light > left_to))//
  {
    for(int x =0; x<8; x++){
        m_light += analogRead(lineSensorM);
        l_light += analogRead(lineSensorL);

    }
    m_light = m_light/8;
    l_light = l_light/8;
   // Serial.println(m_light);
  }
    delay(post_movement_delay);

  right.write(90);
  left.write(90);

  l_light=0;
  m_light=0;
  r_light=0;
}

void pivotRight() 
{
  l_light=0;
  m_light=0;
  r_light=0;

  right.write(slow_R);
  left.write(slow_L);
  delay(500);
  
  //STOP
  right.write(90);
  left.write(90);
  right.write(back_slow_R);
  left.write(slow_L);
  delay(300);

  // DO AN INITIAL READ
  for(int x =0; x<4; x++){
    m_light += analogRead(lineSensorM);
    r_light += analogRead(lineSensorR);
  }
  m_light = m_light/4;
  r_light = r_light/4;
  
  while (!(m_light < mid_to && r_light > right_to))
    {
      for(int x =0; x<8; x++){
        m_light += analogRead(lineSensorM);
        r_light += analogRead(lineSensorR);
    }
    m_light = m_light/8;
    r_light = r_light/8;
   // Serial.println(m_light);
  }
  delay(post_movement_delay);
  right.write(90);
  left.write(90);

  l_light=0;
  m_light=0;
  r_light=0;
}

//fix.
void pivot180(){
    l_light=0;
    m_light=0;
    r_light=0;

    left.write(med_L);//med_fast_L
    right.write(back_med_R);
    delay(500);
    left.write(slow_L);
    right.write(back_slow_R);
    
    while (!(m_light < mid_to && r_light > right_to))
    {
      for(int x=0;x<4;x++){
        l_light += analogRead(lineSensorL);
        m_light += analogRead(lineSensorM);
        r_light += analogRead(lineSensorR);
      }
        l_light = l_light/4;
        m_light = m_light/4;
        r_light = r_light/4;

    }
   delay(post_movement_delay);
   left.write(back_slow_L);
   right.write(slow_R);
   delay(20);

    l_light=0;
    m_light=0;
    r_light=0;
}

/* -------------------------------
 *         SENSOR CHECKS
 * -------------------------------
 */

void checkWall(){
  // Clear the Averagers
    av_Dist_F = 0;
    av_Dist_L = 0;
    av_Dist_R = 0;
  
  // Update all distance sensors.
  for (int x = 0; x < 4; x++){
    av_Dist_F +=  analogRead(DistSensorF);
    av_Dist_L += analogRead(DistSensorL);
    av_Dist_R += analogRead(DistSensorR);
  }
  distLeft = av_Dist_L / 4;
  distFront = av_Dist_F / 4;
  distRight = av_Dist_R / 4;
  
  wallLeft = 0;
  wallRight = 0;
  wallForward = 0;

  if (distFront>frontDist){
    wallForward=1;
  }

  if (distRight>sideDist){
    wallRight=1;
  }

  if (distLeft>sideDist){
    wallLeft=1;
  }
  Serial.println(analogRead(DistSensorR));
  // Serial.print("Front: ");
  // Serial.println(distFront);
  // Serial.print("Right: ");
  // Serial.println(distRight);
}

/* -------------------------------
 *         LINE FOLLOWER 
 * -------------------------------
 */

//SIMPLE CORRECTIVE ALGORITHM:
//When in doubt, check thresholds.
//Then, adjust the sensors to be nicely spaced.
void pd_followLine()
{
    l_light=0;
    m_light=0;
    r_light=0;
    
    tempspeed = 0;

    for(int x=0;x<4;x++){
      l_light += analogRead(lineSensorL);
      m_light += analogRead(lineSensorM);
      r_light += analogRead(lineSensorR);
    }

    l_light = l_light/4;
    m_light = m_light/4;
    r_light = r_light/4;

    if (l_light < left_to && r_light < right_to && m_light < mid_to) {
      right.write(slow_R);
      left.write(slow_L);
    }
    else if (r_light < right_to && l_light > left_to) // Less means on the line
    {
      right.write(med_R);
    }
    else if (l_light < left_to && r_light > right_to)
    {
      left.write(med_L);
    }
    else
    {
      left.write(med_fast_L);
      right.write(med_fast_R);
    }  
    
    // l_light=0;
    // m_light=0;
    // r_light=0;
}

void intersect() {
  pivot180();
}

/* -------------------------------
 *        SEARCH ALGORITHM 
 * -------------------------------
 */

void directionPick(){
  checkWall();
  if(!Maze[xpos][ypos].visited){
    switch(Maze[xpos][ypos].direction)
    { 
        case North: 
            Maze[xpos][ypos].south = 1;
        break;
        case East: 
            Maze[xpos][ypos].west = 1;
        break;
        case West: 
            Maze[xpos][ypos].east = 1;
        break;
        case South: 
            Maze[xpos][ypos].north = 1;
        break;
    }
  }
  switch(recentDirection){
        case North: 
            Maze[xpos][ypos].east |= wallRight;
            Maze[xpos][ypos].west |= wallLeft;
            Maze[xpos][ypos].north |= wallForward; 
            break;
        case East: 
            Maze[xpos][ypos].east |= wallForward;
            Maze[xpos][ypos].north |= wallLeft;
            Maze[xpos][ypos].south |= wallRight;
            break;
        case West: 
            Maze[xpos][ypos].west |= wallForward;
            Maze[xpos][ypos].north |= wallRight;
            Maze[xpos][ypos].south |= wallLeft;
            break;
        case South:
            Maze[xpos][ypos].east |= wallLeft;
            Maze[xpos][ypos].west |= wallRight;
            Maze[xpos][ypos].south |= wallForward;
        break;
    }
    // Checks nearby node for paths to take. Prevents useless looping.
    if (xpos-1 >=0){
      if (Maze[xpos-1][ypos].visited) Maze[xpos][ypos].west =1;
    }
    if (xpos+1 <= 8){
      if (Maze[xpos+1][ypos].visited) Maze[xpos][ypos].east =1;
    }
    if (ypos+1 <= 8){
      if (Maze[xpos][ypos+1].visited) Maze[xpos][ypos].north =1;
    }
    if (ypos-1 >= 0){
      if (Maze[xpos][ypos-1].visited) Maze[xpos][ypos].south =1;
    }

    Maze[xpos][ypos].visited = 1;

    if(Maze[xpos][ypos].north==0){
      nextDirection = North;
      Maze[xpos][ypos].north=1;
    }
    else if(Maze[xpos][ypos].east==0){
      nextDirection = East;
      Maze[xpos][ypos].east=1;
    }
    else if(Maze[xpos][ypos].west==0){
      nextDirection = West;
      Maze[xpos][ypos].west =1;
    }
    else if(Maze[xpos][ypos].south==0){
      nextDirection = South;
       Maze[xpos][ypos].south =1;
    }
    else{
      nextDirection = (Maze[xpos][ypos].direction+2)%4;
    }
}

void moveToNearbyNode(int direction){
  diff = abs(direction - recentDirection);
  if (diff == 0){
    left.write(slow_L);
    right.write(slow_R);
    //delay(post_movement_delay);
    pd_followLine();
    
  }
  else if (diff == 2){
    pivot180();
  }
  else if (recentDirection == (direction+1)%4){
    pivotLeft();
  }
  else if ((recentDirection+1)%4 == direction){
    pivotRight();
  }
  //pd_followLine();
  // else{
  //   pivot180();
  // }

  // increment and update next node.
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

    if (!Maze[xpos][ypos].visited) {
      Maze[xpos][ypos].direction = direction;
    }
    recentDirection = direction;

    while(!(l_light < left_to && r_light < right_to && m_light < mid_to)){    
      pd_followLine();
   }

    // for(int x=0;x<4;x++){
    //   l_light += analogRead(lineSensorL);
    //   m_light += analogRead(lineSensorM);
    //   r_light += analogRead(lineSensorR);
    // }

    // l_light = l_light/4;
    // m_light = m_light/4;
    // r_light = r_light/4;
    // }
    
    left.write(slow_L);
    right.write(slow_R);

}



void setup() {
  Serial.begin(9600);
  left.attach(pinLeft);
  right.attach(pinRight);
  pinMode(2, INPUT);

  // Stop all motors
  left.write(90);
  right.write(90);

}

void loop() {
  // Check Button Press
//  while(1){
//    checkWall();
//  }      

  if (digitalRead(2) == 1){
    start = 1;
    while (!(l_light < left_to && r_light < right_to && m_light < mid_to)){
      pd_followLine();
      for(int x=0;x<4;x++){
        l_light += analogRead(lineSensorL);
        m_light += analogRead(lineSensorM);
        r_light += analogRead(lineSensorR);
        }
    
        l_light = l_light/4;
        m_light = m_light/4;
        r_light = r_light/4;     
    }
    
  }
  if(start){
    while(1) {
     
      directionPick();
      moveToNearbyNode(nextDirection);

    //  pd_followLine();
    // for(int x=0;x<4;x++){
    //   l_light += analogRead(lineSensorL);
    //   m_light += analogRead(lineSensorM);
    //   r_light += analogRead(lineSensorR);
    // }

    // l_light = l_light/4;
    // m_light = m_light/4;
    // r_light = r_light/4;
    //  if (l_light < left_to && r_light < right_to && m_light < mid_to) {
    //      intersect();
    //      Serial.println("here");
   }
  }
}
