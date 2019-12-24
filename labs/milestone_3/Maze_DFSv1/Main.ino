//#include <Maze_DFS.c>
//#include <Robot_Move.c>
 #include <Servo.h>

 #include <stdio.h>
 #include <stdlib.h>

#define byte uint8_t

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
  
  //Motor Setup
  leftStop();
  rightStop();
  
  DFSInit();
}

void loop()
{ 
  if (start){
    checkIntersect();
    if(intersectFlag) {
      DFS(0);
      intersectFlag = 0;
    }
  }

}
