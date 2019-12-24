#include <Servo.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"


typedef uint8_t byte;

int start = 0;
int done = 0;

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
#define left_to 450     
#define mid_to 450
#define right_to 450

#define post_movement_delay 50

// Distance Sensor Thresholds
#define sideDist 250    // Above is wall.
#define frontDist 175 
#define alleyDist 250

// turning thresholds
#define pivot_overshoot 250
#define turn_check_overshoot 200
#define check_line_delay 20

// Robot Detection Thresholds
#define irFrontThreshold 800 //todo calibrate


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
int mux_c = 3;
int mux_b = 4;
int mux_a = 7;
int mux_out = A1;
int distFront = 0;
int distLeft = 0;
int distRight = 0;

// robot sensor
int irFront = 0; 
int av_irFront = 0; 
bool robotFront = 0;

int greenLED = 8;
int redLEDs = 1;


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

unsigned int walls;
unsigned int data;

RF24 radio(9,10);

//
// Topology
//

// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = { 0x0000000046LL, 0x0000000047LL };


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
    int test = 5;
    while (!(m_light < mid_to && r_light > right_to))
    {
      for(int x=0;x<test;x++){
        l_light += analogRead(lineSensorL);
        m_light += analogRead(lineSensorM);
        r_light += analogRead(lineSensorR);
      }
        l_light = l_light/test;
        m_light = m_light/test;
        r_light = r_light/test;

    }
   delay(post_movement_delay);
   //left.write(back_slow_L);
   //right.write(slow_R);
   //delay(35);

    l_light=0;
    m_light=0;
    r_light=0;
}

void pivot180detection(){
    l_light=0;
    m_light=1000;
    r_light=0;

    left.write(med_L);//med_fast_L
    right.write(back_med_R);
    //left.write(slow_L);
    //right.write(back_slow_R);
    delay(1500);
    //left.write(slow_L);
    //right.write(back_slow_R);
    int test = 4;
    while (!(m_light < mid_to))
    {
      m_light=0;
      for(int x=0;x<test;x++){
        m_light += analogRead(lineSensorM);
      }
        m_light = m_light/test;
    }
   Serial.println("-----");
   Serial.println(m_light);
   delay(post_movement_delay);
   //left.write(back_slow_L);
   //right.write(slow_R);
   //delay(35);
    
}

void pivot180detection_alt(){
    l_light=0;
    m_light=1000;
    r_light=0;

    left.write(back_med_L);//med_fast_L
    right.write(med_R);
    delay(1500);
    //left.write(back_slow_L);
    //right.write(slow_R);
    int test = 4;
    while (!(m_light < mid_to))
    {
      m_light=0;
      for(int x=0;x<test;x++){
        m_light += analogRead(lineSensorM);
      }
        m_light = m_light/test;
    }
   delay(post_movement_delay);
   Serial.println(m_light);
   //left.write(back_slow_L);
   //right.write(slow_R);
   //delay(35);
}

void pivotRightatStart() 
{
  l_light=0;
  m_light=0;
  r_light=0;
  
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

/* -------------------------------
 *         SENSOR CHECKS
 * -------------------------------
 */

void checkWall(){
  // Clear the Averagers
    av_Dist_F = 0;
    av_Dist_L = 0;
    av_Dist_R = 0;

  //CBA = pin3,pin4,pin7
  // Update all distance sensors.
  for (int x = 0; x < 4; x++){
    //front 
    digitalWrite(mux_c, HIGH); 
    digitalWrite(mux_b, LOW); 
    digitalWrite(mux_a, LOW); 
    // |= 0b00001000;  
    //PORTD &= 0b01101111;
    av_Dist_F +=  analogRead(mux_out);

    //left
    digitalWrite(mux_c, LOW);       
    digitalWrite(mux_b, LOW);       
    digitalWrite(mux_a, LOW);
    //PORTD &= 0b01100111;   
    av_Dist_L += analogRead(mux_out);

    //right
    digitalWrite(mux_c, LOW);       
    digitalWrite(mux_b, HIGH);       
    digitalWrite(mux_a, LOW); 
    //PORTD &= 0b01110111; 
    //PORTD |= 0b00010000;  
    av_Dist_R += analogRead(mux_out);
  }
  distLeft = av_Dist_L / 4;
  distFront = av_Dist_F / 4;
  distRight = av_Dist_R / 4;
  
  wallLeft = 0;
  wallRight = 0;
  wallForward = 0;

  if (distRight>sideDist){
    wallRight=1;
  }

  if (distLeft>sideDist){
    wallLeft=1;
  }

  if(wallRight&&wallLeft){
    if(distFront>alleyDist){
      wallForward = 1;
    }
  }
  else{
    if(distFront>frontDist){
      wallForward=1;
    }
  }
//  Serial.println(distLeft);
//  Serial.println(distFront);
//  Serial.println(distRight);
//  Serial.println("---------------------------");

   //Serial.print("Front: ");
   //Serial.println(wallForward);
   //Serial.print("Right: ");
   //Serial.println(wallRight);

     //Serial.println("---------------------------");

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

    for(int x=0;x<5;x++){
      l_light += analogRead(lineSensorL);
      m_light += analogRead(lineSensorM);
      r_light += analogRead(lineSensorR);
    }

    l_light = l_light/5;
    m_light = m_light/5;
    r_light = r_light/5;


    //Serial.println(m_light);

    if (l_light < left_to && r_light < right_to && m_light < mid_to) {
      right.write(slow_R);
      left.write(slow_L);
    }
    else if (r_light < right_to && l_light > left_to) // Less means on the line
    {
      right.write(med_R);
      left.write(med_fast_L);

    }
    else if (l_light < left_to && r_light > right_to)
    {
      left.write(med_L);
      right.write(med_fast_R);

    }
    else
    {
      left.write(med_fast_L);
      right.write(med_fast_R);
    }  
}

void pd_followLineSlow()
{
    l_light=0;
    m_light=0;
    r_light=0;
    
    tempspeed = 0;

    for(int x=0;x<5;x++){
      l_light += analogRead(lineSensorL);
      m_light += analogRead(lineSensorM);
      r_light += analogRead(lineSensorR);
    }

    l_light = l_light/5;
    m_light = m_light/5;
    r_light = r_light/5;


    //Serial.println(m_light);

    if (l_light < left_to && r_light < right_to && m_light < mid_to) {
      right.write(slow_R);
      left.write(slow_L);
    }
    else if (r_light < right_to && l_light > left_to) // Less means on the line
    {
      right.write(slow_R);
      left.write(med_L);

    }
    else if (l_light < left_to && r_light > right_to)
    {
      left.write(slow_L);
      right.write(med_R);

    }
    else
    {
      left.write(slow_L);
      right.write(slow_R);
    }  
}

void pd_reverseLine()
{
    l_light=0;
    m_light=0;
    r_light=0;
    
    tempspeed = 0;

    for(int x=0;x<5;x++){
      l_light += analogRead(lineSensorL);
      m_light += analogRead(lineSensorM);
      r_light += analogRead(lineSensorR);
    }

    l_light = l_light/5;
    m_light = m_light/5;
    r_light = r_light/5;


    //Serial.println(m_light);

    if (l_light < left_to && r_light < right_to && m_light < mid_to) {
      right.write(back_slow_R);
      left.write(back_slow_L);
    }
    //veering left
    else if (r_light < right_to && l_light > left_to) // Less means on the line
    {
      right.write(back_med_fast_R);
      left.write(back_med_L);

    }
    //veering right
    else if (l_light < left_to && r_light > right_to)
    {
      left.write(back_med_fast_L);
      right.write(back_med_R);

    }
    else
    {
      left.write(back_med_fast_L);
      right.write(back_med_fast_R);
    }  
}

void intersect() {
  pivot180();
}

/* -------------------------------
 *         ROBOT DETECTION
 * -------------------------------
 */

/*Checks for Robot in front of path. Sets robotFront to 1 if true, else 0.*/
void checkRobot() {
  // irFront = 0;
  av_irFront = 0;
  for (int i = 0; i < 4; i++) {
    digitalWrite(mux_c, LOW);       
    digitalWrite(mux_b, HIGH);       
    digitalWrite(mux_a, HIGH); 
    av_irFront += analogRead(mux_out);
  }
  irFront = av_irFront / 4;
//  Serial.println(irFront);
  robotFront = 0;
  if (irFront >= irFrontThreshold) {
    robotFront = 1;
  }
}

/* -------------------------------
 *        SEARCH ALGORITHM 
 * -------------------------------
 */

void directionPick(){
  left.write(slow_L);
  right.write(slow_R);

  Serial.print("(");
  Serial.print(xpos);
  Serial.print(",");
  Serial.print(ypos);
  Serial.println(")");
  
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
  
  walls = 0;
  data = 0;
  switch(recentDirection){
        case North: 
            Maze[xpos][ypos].east |= wallRight;
            Maze[xpos][ypos].west |= wallLeft;
            Maze[xpos][ypos].north |= wallForward; 
            walls |= (wallLeft<<0);
            walls |= (wallForward<<3);
            walls |= (wallRight<<2);
            break;
        case East: 
            Maze[xpos][ypos].east |= wallForward;
            Maze[xpos][ypos].north |= wallLeft;
            Maze[xpos][ypos].south |= wallRight;
            walls |= (wallLeft<<3);
            walls |= (wallForward<<2);
            walls |= (wallRight<<1);
            break;
        case West: 
            Maze[xpos][ypos].west |= wallForward;
            Maze[xpos][ypos].north |= wallRight;
            Maze[xpos][ypos].south |= wallLeft;
            walls |= (wallLeft<<1);
            walls |= (wallForward<<0);
            walls |= (wallRight<<3);
            break;
        case South:
            Maze[xpos][ypos].east |= wallLeft;
            Maze[xpos][ypos].west |= wallRight;
            Maze[xpos][ypos].south |= wallForward;
            walls |= (wallLeft<<2);
            walls |= (wallForward<<1);
            walls |= (wallRight<<0);
        break;
    }

    data = (walls<<8)|(ypos<<4)|(xpos);
    //unsigned int data2 = (B00001111 * 256) + B00000011;
    radio.startWrite(&data, sizeof(unsigned int));

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

    if (xpos==0 && ypos==0){
      if (Maze[xpos][ypos].north && Maze[xpos][ypos].east && (recentDirection==2 || recentDirection==3)){
          left.detach();
          right.detach();
          digitalWrite(greenLED,HIGH);
          done=1;
          return;
      }
    }
}

void moveToNearbyNode(int direction){
  //left.write(slow_L);
  //right.write(slow_R);
  //delay(50);
  diff = abs(direction - recentDirection);
  if (diff == 0){
    left.write(slow_L);
    right.write(slow_R);
    //delay(post_movement_delay);
    for (int i=0;i<150;i++){
      pd_followLine();
    }
    Serial.println("fwd");
    
  }
  else if (diff == 2){
    pivot180();
    Serial.println("back");
  }
  else if (recentDirection == (direction+1)%4){
    pivotLeft();
    Serial.println("left");
  }
  else if ((recentDirection+1)%4 == direction){
    if (xpos==0 && ypos==0 && recentDirection==0){
      pivotRightatStart();
    }
    else{
      pivotRight();
    }
    Serial.println("right");
  }
  pd_followLine();
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

    // **** robot detection ****   todo insert this snippet into final integration

    while(!(l_light < left_to && r_light < right_to && m_light < mid_to)){
        pd_followLine();
        checkRobot();
        while (robotFront==1){
          digitalWrite(redLEDs, HIGH);
          pivot180detection();
          pivot180detection_alt();
          checkRobot();
        }
        digitalWrite(redLEDs, LOW);
      }
      
      // **** robot detection ****    todo
      
    
    
    left.write(slow_L);
    right.write(slow_R);
    delay(60);
}



void setup() {
  //
  // Setup and configure rf radio
  //

  radio.begin();

  // optionally, increase the delay between retries & # of retries
  radio.setRetries(15,15);
  radio.setAutoAck(true);
  // set the channel
  radio.setChannel(0x50);
  // set the power
  // RF24_PA_MIN=-18dBm, RF24_PA_LOW=-12dBm, RF24_PA_HIGH=-6dBM, and RF24_PA_MAX=0dBm.
  radio.setPALevel(RF24_PA_MAX);
  //RF24_250KBPS for 250kbs, RF24_1MBPS for 1Mbps, or RF24_2MBPS for 2Mbps
  radio.setDataRate(RF24_250KBPS);

  // optionally, reduce the payload size.  seems to
  // improve reliability
  //radio.setPayloadSize(8);

  //
  // Open pipes to other nodes for communication
  //

  // This simple sketch opens two pipes for these two nodes to communicate
  // back and forth.
  // Open 'our' pipe for writing
  // Open the 'other' pipe for reading, in position #1 (we can have up to 5 pipes open for reading)

  radio.openWritingPipe(pipes[0]);
  radio.openReadingPipe(1,pipes[1]);
  
  //
  // Stop listening
  //
  //May be unnecessary
  radio.startListening();
  radio.stopListening();

  
  Serial.begin(9600);
  left.attach(pinLeft);
  right.attach(pinRight);
  pinMode(2, INPUT);


  // Mux setup
  pinMode(mux_c,OUTPUT);
  pinMode(mux_b,OUTPUT);
  pinMode(mux_a,OUTPUT);

  pinMode(greenLED,OUTPUT);
  pinMode(redLEDs,OUTPUT);

  digitalWrite(redLEDs, LOW);
  

  // Stop all motors
  left.write(90);
  right.write(90);

}

void loop() {
  // Check Button Press
//  while(1){
//    checkWall();
//      checkRobot();
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
      if (done){
        return;
      }
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
