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
int mux_c = 3;
int mux_b = 4;
int mux_a = 7;
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


// light pins
int ledPin_front = 2;
int ledPin_right = 3;

int travelCounter;
int nextDirection;

int m = 0;

long averager = 0;
int sample = 50;

void buff_clear(){
  while(Serial.available() != 0){
    Serial.read();
  }
}

void setup(){
  Serial.begin(9600);
  
  pinMode(mux_c,OUTPUT);
  pinMode(mux_b,OUTPUT);
  pinMode(mux_a,OUTPUT);
}

void loop(){

//  while(1){
//    averager = analogRead(A1);
//    Serial.println(averager);
//  }
  int state_flag = 0;
  averager = 0;
  Serial.print("\nPlace front wall in front of choice location! Press 1.\n");
  while(state_flag != 1){
    while(Serial.available() == 0) { }
    if (Serial.available() > 0){
      int readByte = Serial.parseInt();
      if (readByte ==  1){
        digitalWrite(mux_c, HIGH);       
          digitalWrite(mux_b, LOW);       
          digitalWrite(mux_a, LOW); 
        for(int x =0; x<sample; x++){
            
          averager += analogRead(A1);
        }
        int front_averaged = averager/sample;
        state_flag = 1;
        Serial.print("Ah yes, it's done.\n");
        Serial.print(front_averaged);
        Serial.println("");
      }
      else{
        Serial.print("1 not pressed. Try again.\n");
      }
      buff_clear();
    }
  }
  

}
