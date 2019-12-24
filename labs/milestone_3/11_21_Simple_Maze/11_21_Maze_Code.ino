#include <Servo.h>

typedef uint8_t byte;

int start = 0;

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

// int nextMove = 0;

bool wallLeft = 0;
bool wallRight = 0;
bool wallForward = 0;


/* Check if there are walls around the robot; make it react accordingly */
void checkWall() {
  checkDistSensors();

  //default to no surrounding walls
  wallLeft = 0;
  wallRight = 0;
  wallForward = 0;

  //front sensor detects wall, set wall value with shift by compass to reorient
  if (distFront>frontDist){
    wallForward=1;
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
    // Serial.print("Front: ");
    // Serial.println(wallForward);
//    Serial.print("Right: ");
//    Serial.println(wallRight);
}


////////////////////////////////////////////////
/*----------------- MAZE  -----------------*/
////////////////////////////////////////////////


// Called after wallcheck and determining unvisited node.
void moveToNearbyNode(int direction){
    intersectFlag = 0;    
    // Turn
    diff = abs(direction - recentDirection); // previously direction - Maze[xpos][ypos].direction
    if (diff == 0){
        //If shit goes wrong come back here!!!!
        Serial.println("forward");
//        int counter = 0;
//        while (counter < 50){
            pd_followLine();
//            counter +=1;
//        }
    }
    else if (diff == 2){
        Serial.println("back");
        pivot180();
//        pd_followLine();
    //   if (PIND|=(1<<2)){
    //       PORTD^=(1<<2);// toggle on pin 2 led
    //   }
    //   if (PIND|=(1<<3)){
    //       PORTD^=(1<<3);// toggle on pin 3 led
    //   }
    }
    else if (recentDirection == (direction+1)%4){
        Serial.println("left");
        pivotLeft();
//        pd_followLine();
    }
    else if ((recentDirection+1)%4 == direction){
        Serial.println("right");
        pivotRight();
//        pd_followLine();
    }
//    Serial.print("Direction:");
//    Serial.println(direction);
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
    
//    Serial.print("****************");
//    Serial.print("(");Serial.print(xpos);Serial.print(",");Serial.print(ypos);Serial.println(")");
   
    if (!Maze[xpos][ypos].visited) Maze[xpos][ypos].direction = direction;
    recentDirection = direction;
//     Serial.print("Direction:");
//     Serial.println(Maze[xpos][ypos].direction);
    // Move to next intersection  
    while(!intersectFlag){
      pd_followLine();
//      checkIntersect();
        checkLineSensors();
    }
    left.write(90);
    right.write(90);
}
// Decision making method.
void directionPick(){
    checkWall();
    // Check sensors and see what's on the market.
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
//    Serial.print(" N:");
//    Serial.print(Maze[xpos][ypos].north);
//    Serial.print(" E:");
//    Serial.print(Maze[xpos][ypos].east);
//    Serial.print(" S:");
//    Serial.print(Maze[xpos][ypos].south);
//    Serial.print(" W:");
//    Serial.println(Maze[xpos][ypos].west);
    
    Maze[xpos][ypos].visited = 1;
//    Serial.print("Node is now visited:");
//    Serial.println(Maze[xpos][ypos].visited);
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
//      Serial.println("backTrack");
      nextDirection = (Maze[xpos][ypos].direction+2)%4;
//      PORTD|=(1<<2);// turn on pin 2 led
//      PORTD|=(1<<3);// turn on pin 2 led
    }
//    return nextMove;
}


void setup() {
  Serial.begin(9600);
  left.attach(pinLeft);
  right.attach(pinRight);
  pinMode(13, INPUT);

  // Stop all motors
  left.write(90);
  right.write(90);

  Maze[0][0].south = 1;

}

void loop(){
if (digitalRead(13) == 1){
    start = 1;
    Serial.println(start);
  }

if(start){
//    nextDirection =0;
   directionPick();
   moveToNearbyNode(nextDirection);
//    checkLineSensors();
//    checkDistSensors();
//  pd_followLine();
//  rightHandFollow();
}