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
#define left_to 283     
#define mid_to 328
#define right_to 311

// Distance Sensor Thresholds
#define distLow 200     //far
#define distHigh 650    //close
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
int DistSensorM = A2;
int DistSensorR = A1;
int distFront = 0;
int distLeft = 0;
int distRight = 0;


// BELOW THIS  IT IS NOT USED
// Large print-out.
int travelCounter;
int nextDirection;

int m = 0;
// ------------------------------

/* -------------------------------
 *        FXN DEFINITIONS
 * -------------------------------
 */
 int tempspeed;

/* -------------------------------
 *         SENSORS 
 * -------------------------------
 */

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
  right.write(slow_R);
  left.write(slow_L);
  delay(500);
  
  //STOP
  right.write(90);
  left.write(90);
  right.write(slow_R);
  left.write(back_slow_L);
  delay (300);

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
    delay (50);

  right.write(90);
  left.write(90);
}

void pivotRight() 
{
  right.write(slow_R);
  left.write(slow_L);
  delay(500);
  
  //STOP
  right.write(90);
  left.write(90);
  right.write(back_slow_R);
  left.write(slow_L);
  delay (300);

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
  delay (50);
  right.write(90);
  left.write(90);
}

//fix.
void pivot180(){
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
   delay (50);
   right.write(90);
   left.write(90);
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
    tempspeed = 0;

    for(int x=0;x<4;x++){
      l_light += analogRead(lineSensorL);
      m_light += analogRead(lineSensorM);
      r_light += analogRead(lineSensorR);
    }

    l_light = l_light/4;
    m_light = m_light/4;
    r_light = r_light/4;
//[
    if (l_light < left_to && r_light < right_to && m_light < mid_to) {
      right.write(slow_R);
      left.write(med_R);
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
}

void intersect() {
  pivot180();
}

void setup() {
  Serial.begin(9600);
  left.attach(pinLeft);
  right.attach(pinRight);
  pinMode(13, INPUT);

  // Stop all motors
  left.write(90);
  right.write(90);

}

void loop() {
  // Check Button Press
          

  if (digitalRead(13) == 1){
    start = 1;
  }
  if(start){
    while(1) {
     pd_followLine();
    for(int x=0;x<4;x++){
      l_light += analogRead(lineSensorL);
      m_light += analogRead(lineSensorM);
      r_light += analogRead(lineSensorR);
    }

    l_light = l_light/4;
    m_light = m_light/4;
    r_light = r_light/4;
     if (l_light < left_to && r_light < right_to && m_light < mid_to) {
         intersect();
         Serial.println("here");
     }
    }
  }
}
