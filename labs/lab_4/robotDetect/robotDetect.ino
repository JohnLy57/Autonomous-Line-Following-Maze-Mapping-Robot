#define irFrontThreshold 150 //todo calibrate
// #define irBackThreshold 500
// #define irLeftThreshold 500
// #define irRightThreshold 500

/* robot detection sensors */
int irSensorFront = A1; //A1, A2, 3, 4, 7 are available
// int irSensorLeft = A1;
// int irSensorRight = A2;
// int irSensorBack = A3; // random pins
int irFront = 0; 
int av_irFront = 0; 
// int irBack = 0;
// int irLeft = 0;
// int irRight = 0;
bool robotFront = 0;


void checkRobot() {
  irFront = 0;
  av_irFront = 0;
  int n = 4;
  for (int i = 0; i < n; i++) {
    av_irFront += analogRead(irSensorFront);
  }
  irFront = av_irFront / n;

  robotFront = 0;
  if (irFront >= irFrontThreshold) {
    robotFront = 1;
  }
}


/*************************
 * put robot detection code (marked by todo) into this function in the final integration
 * *******************************/ 
void moveToNearbyNode(int direction){
  diff = abs(direction - recentDirection);
  if (diff == 0){
    left.write(slow_L);
    right.write(slow_R);
    //delay(post_movement_delay);
    for (int i=0;i<150;i++){
      pd_followLine();
    }
    
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

    while(!(l_light < left_to && r_light < right_to && m_light < mid_to)){    
      pd_followLine();

      // **** robot detection ****   todo insert this snippet into final integration
      int t = 700;
      int totalTime = 0;
      int totalTimeLimit = 10000;
      checkRobot();
      while (robotFront) {
        if (totalTime >= totalTimeLimit) {
          break;
        }

        left.write(back_slow_L);
        right.write(back_slow_R);
        delay(t); //todo drift should try pd_followLine
        totalTime += t;
        left.write(90);
        right.write(90);

        // left.write(slow_L);
        // right.write(slow_R);
        // delay(t);
        // totalTime += t;

        for (int i = 0; i < t * 20; i++) {
          pd_followLine();
        }
        left.write(90);
        right.write(90);
        checkRobot();
      }
      // **** robot detection ****    todo
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
    //delay(50);

}

void setup() {
  

}

void loop() {
  
}
