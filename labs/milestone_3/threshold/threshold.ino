#include <Servo.h>

typedef uint8_t byte;

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
#define left_to;
#define mid_to;
#define right_to;

// dist sensors thresholds
#define distLow;    //far
#define distHigh;   //close
#define frontDist;   

// turning thresholds

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

void setup(){
  long averager = 0;
}

void loop(){
  int counter = 50;
  while (counter < 50){
    analogRead(A5);
    
  }
}
