#include "stdlib.h"
#include <Servo.h>

/* -------------------------------
 *        Hardware
 * -------------------------------
 */

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

#define left_to 500
#define mid_to 500
#define right_to 500

// dist sensors thresholds
#define distLow 180  //far
#define distHigh 575 //close

#define frontDistLow 180  //far
#define frontDistHigh 280 //close

int sampler = 5;
int sampler2 = 5; //8

int start = 0;

Servo left;
Servo right;
int pinLeft = 5;
int pinRight = 6;
int lineSensorL = A0;
int lineSensorM = A1;
int lineSensorR = A2;
int LSL;
int LSM;
int LSR;
int intersectFlag = 0;
int turnDirection = 0; // 0 is left, 1 is right.

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

int ledPin_front = 2;
int ledPin_right = 3;

void leftForward(int speed)
{
    left.write(speed);
}

void rightForward(int speed)
{
    right.write(speed);
}

void leftStop()
{
    left.write(90);
}

void rightStop()
{
    right.write(90);
}

void robotForward()
{
    leftForward(med_fast_L);
    rightForward(med_fast_R);
}

void robotStop()
{
    leftStop();
    rightStop();
    //delay(200);
}

void pd_followLine()
{
    //working slow settings:
    // 0.9, 85 +/- 5

    int KD = 0.9;
    int tempspeed;

    checkSensors();
    // right drifts onto white
    if (LSR < 500)
    {
        int rdiff = (850 - LSR) / 850;
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
    checkSensors();
    if (LSL < left_to && LSR < right_to)
    {
        intersectFlag = 1;
    }
}

void pivotLeft()
{
    rightForward(med_R);
    leftForward(med_L);
    delay(300);
    robotStop();
    rightForward(med_R);
    leftForward(back_med_L);
    delay(300);

    int ready = 0;
    while (ready != sampler2)
    {
        ready += checkLine();
    }
    intersectFlag = 0;
}

void pivotRight()
{
    rightForward(med_R);
    leftForward(med_L);
    delay(300);
    robotStop();
    leftForward(med_L);
    rightForward(back_med_R);
    delay(300);

    int ready = 0;
    while (ready != sampler2)
    {
        ready += checkLine();
    }
    intersectFlag = 0;
}

/* Check whether robot is on a straight line;
   Return 1 if true, 0 if false */
int checkLine()
{
    checkSensors();
    if (LSL > left_to && LSM < mid_to && LSR > right_to)
    {
        return 1;
    }
    else
        return 0;
}

/* Get the averaged readings from line sensors (for line following) */
void checkSensors()
{
    int av_L = 0;
    int av_R = 0;
    int av_M = 0;
    int sampler = 4;

    for (int x = 0; x < sampler; x++)
    {
        av_L += analogRead(lineSensorL);
        av_M += analogRead(lineSensorM);
        av_R += analogRead(lineSensorR);
    }

    LSL = av_L / sampler;
    LSM = av_M / sampler;
    LSR = av_R / sampler;
}

/* Get the readings from distance sensors (for wall detection)*/
void checkDistSensors()
{
    int ready = 0;
    while (ready < sampler)
    {

        // SWEEP PIN:
        // FRONT MUX = 2
        // RIGHT MUX = 1
        // BACK MUX =  0
        digitalWrite(mux_a, LOW);
        digitalWrite(mux_b, HIGH);
        digitalWrite(mux_c, LOW);
        distFront += analogRead(A3);
        digitalWrite(mux_a, LOW);
        digitalWrite(mux_b, LOW);
        digitalWrite(mux_c, LOW);
        distLeft += analogRead(A3);
        digitalWrite(mux_a, HIGH);
        digitalWrite(mux_b, LOW);
        digitalWrite(mux_c, LOW);
        distRight += analogRead(A3);
        ready += 1;
    }
    distFront = distFront / sampler;
    distLeft = distLeft / sampler;
    distRight = distRight / sampler;
}

/* Get the readings from IR sensors (for robot detection)*/
void checkIR()
{
    // FRONT MUX = 4
    // RIGHT MUX = 6
    // LEFT MUX = 7
    // AFT MUX = 5
    digitalWrite(mux_a, LOW);
    digitalWrite(mux_b, LOW);
    digitalWrite(mux_c, HIGH);
    irFront = analogRead(A3);
    digitalWrite(mux_a, LOW);
    digitalWrite(mux_b, HIGH);
    digitalWrite(mux_c, HIGH);
    irRight = analogRead(A3);
    digitalWrite(mux_a, HIGH);
    digitalWrite(mux_b, HIGH);
    digitalWrite(mux_c, HIGH);
    irLeft = analogRead(A3);
    digitalWrite(mux_a, HIGH);
    digitalWrite(mux_b, LOW);
    digitalWrite(mux_c, HIGH);
    irBack = analogRead(A3);
}

void pivot180()
{
    leftForward(med_fast_L);
    rightForward(back_med_fast_R);
    delay(400);
    int ready = 0;
    while (ready != sampler2)
    {
        ready += checkLine();
    }
    intersectFlag = 0;
}

void spinner()
{
    pd_followLine();
    checkIntersect();
    if (intersectFlag)
    {
        pivot180();
    }
}



/* -------------------------------
 *        DFS
 * -------------------------------
 */

#define N 100    // number of nodes
#define xSize 10 // horizontal length of maze
#define ySize 10 // vertical length of maze
#define initDir 0 //initial direction the robot is facing

struct AdjNode {
    int ind;
    int x;
    int y;
    int visited; //0 if not, 1 if yes
    int neighbors[4]; // -1 means no neighbor
    int edges[4]; // -1 means wall, 1 means normal edge (no wall)
    struct AdjNode * prev; // pointer to previous node on robot path
};

typedef struct AdjNode Node;

// global vars
Node * nodes[xSize][ySize];
int dirs[4][2] = {{1, 0}, {0, -1}, {-1, 0}, {0, 1}};
int currX;
int currY;
int currNode;
int currDir;
Node * path; // points to the head of the list, which is the current node
int isOver; //maze traversal ends if = 1, else = 0

int getX(int ind) {
    return ind % xSize;
}

int getY(int ind) {
    return ind / xSize;
}

int getInd(int x, int y) {
    return xSize * y + x;
}

//todo: order of neighbors[] determines robot's DFS priority
void createNode(int x, int y) {
    Node *newNode = (Node *)malloc(sizeof(Node));
    newNode->ind = getInd(x, y);
    newNode->x = x;
    newNode->y = y;
    newNode->visited = 0;
    newNode->prev = NULL;

    for (int j = 0; j < 4; j++) {
            int x1 = x + dirs[j][0];
            int y1 = y + dirs[j][1];
            if (x1 < 0 || x1 >= xSize || y1 < 0 || y1 >= ySize) {
                newNode->neighbors[j] = -1;
                newNode->edges[j] = -1;
            } else {
                newNode->neighbors[j] = getInd(x1, y1);
                newNode->edges[j] = 1;
            }
        }

    nodes[x][y] = newNode;
}

/** Add new head (current node) to front of the list */
void addToPath(int x, int y) {
    Node * node = nodes[x][y];
    node->prev = path;
    path = node;
}

/** The previous node of the current head becomes the new head */
int backtrack() {
    if (path->prev != NULL) {
        path = path->prev;
        return 1;
    } else {
        return 0; // no node to backtrack to
    }
}

void DFSInit() {
    for (int i = 0; i < xSize; i++) {
        for (int j = 0; j < ySize; j++) {
            createNode(i, j);
        }
    }
    currX = 0;
    currY = 0;
    currNode = 0;
    currDir = initDir;
    path = nodes[0][0];
    isOver = 0;
}

/** Reorient the robot based on its next destination */
void reorient(int currDir, int nextDir) {
    if (nextDir - currDir == 1 || nextDir - currDir == -3) {
        pivotLeft();
    } else if (nextDir - currDir == 2 || nextDir - currDir == -2) {
        pivot180();
    } else if (nextDir - currDir == -1 || nextDir - currDir == 3) {
        pivotRight();
    }
}

/** Use the sensors to check the walls around [currNode] and set currNode->edges[] accordingly*/
void checkWall() {
    checkDistSensors();

    //front sensor detects wall
    if (distFront < distHigh && distFront > distLow) {
        path->edges[currDir] = -1;
    }
    //right sensor detects wall
    if (distRight < distHigh && distRight > distLow) {
        path->edges[(currDir + 1) % 4] = -1;
    }
    //left sensor detects wall
    if (distLeft < distHigh && distLeft > distLow) {
        path->edges[(currDir - 1) % 4] = -1;
    }
}

/** One iteration of the DFS (repeat at each intersection) */
void DFShelper() {
    if (nodes[currX][currY]->visited == 0) {
        checkWall();
        nodes[currX][currY]->visited = 1;
    }
    
    for (int j = 0; j < 4; j++) {
        if (path->edges[j] != -1) {
            int w = path->neighbors[j];
            int x = getX(w);
            int y = getY(w);
            if (nodes[x][y]->visited == 0) {
                reorient(currDir, j);
                currDir = j;
                currX += dirs[j][0];
                currY += dirs[j][1];
                currNode = getInd(currX, currY);
                addToPath(currX, currY);
                return;
            }
        }
    }  

    // backtrack if al neighbors have been visited
    isOver = ! (backtrack());
    if (isOver) {
        return;
    }
    int dirX = path->x - currX;
    int dirY = path->y - currY;
    int nextDir = 0;
    if (dirX == -1) {
        nextDir = 2;
    } else if (dirY == -1) {
        nextDir = 1;
    } else if (dirY == 1) {
        nextDir = 3;
    }
    reorient(currDir, nextDir);
    currDir = nextDir;
    currX = path->x;
    currY = path->y;
    currNode = path->ind;
}

void DFS() {
    while (1) {
        DFShelper();
        if (isOver) { //terminate maze traversal if no more node to backtrack to
            pivot180();
            return;
        }
        // go the newly marked [currNode] by going straight to next intersection
        intersectFlag = 0;
        do {
            pd_followLine();
            checkIntersect(); 
        } while (intersectFlag != 1);
    }
}

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

void loop() {
    //push button start
    if (digitalRead(13) == 1){
        start = 1;
        delay(250);
    }

    if (start) {
        //DFS();
    }
}
