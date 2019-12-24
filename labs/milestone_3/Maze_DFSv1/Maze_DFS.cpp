// #include <stdio.h>
// #include <stdlib.h>
//#include <Robot_Move.c>

#define n 100

//orientation of bot (0-3 = N-W)
enum compass{
  North,
  East,
  South,
  West
};

typedef struct {
  int vertex;
  byte x;
  byte y;
  byte walls; //write to walls if unvisited node
  int adjNodes[4]; //nodes vertex values. 0-3 <=> N-W
  int back;
  // struct node *next
} node;

byte compass=North; //default north to start, bottom left corner
byte back; //direction behind bot;
byte diff = 0; //difference between compass and direction
int currentVertex = 0; //vertex 1-100
int xpos,ypos = 0; //default to (0,0), bottom left corner

node *G[n];

//array containing values for adjacent verticies based on vertex numbers
// int adj[100][4]; 

/* Setup for DFS */
void DFSInit(){
    for(int i=0;i<n;i++){
        G[i]=NULL;
    }
}


/* Updates compass for turns. 0=left turn. 1=right turn. */
void updateCompass(int turn){
  // right turn
  if (turn){
    compass++;
    if (compass>3) compass=0; // W to N, 
  }
  // left turn
  if (!turn){
    compass--;
    if (compass<0) compass=3; // N to W,
  }
}

/* Updates currentVertex with respect to the grid, 0-99 */
void updateVertex(){
  currentVertex = xpos + ypos*10;
}

void addBackNode(node currNode){
  switch(compass){
    case North: back = South;
    case East: back = West;
    case South: back = North;
    case West: back = East;
  }
}

/* Adds Vertex values of all adjacent nodes to current node.
Does not create new nodes. */
void addAllEdges(node currNode){
  checkWall();
  //no wall north
  currNode.adjNodes[0] = (!(walls & 0b00001000)) ? (currentVertex - 10) : -1;
  //no wall east
  currNode.adjNodes[1] = (!(walls & 0b00000100)) ? (currentVertex + 1) : -1;
  //no wall south
  currNode.adjNodes[2] = (!(walls & 0b00000010)) ? (currentVertex + 10) : -1;
  //no wall west  
  currNode.adjNodes[3] = (!(walls & 0b00000001)) ? (currentVertex - 1) : -1;
}

/* Changes direction to get to new edge using compass and direction. 
Moves to new node until intersect is found. */
void moveToNearbyNode(int direction){
  intersectFlag = 0;
  diff = abs(compass - direction);
  if (diff == 2) { //do a 180
    pivotRight();
    pivotRight();
  }
  if (compass == (direction+1)) turnLeft();
  if (compass == (direction-1)) turnRight();
  while(!intersectFlag){
    checkIntersect();
    pd_followLine();
  }
}

void pathToNode(node target){

}

/* Depth First Search traversal on a starting point. 
DFS should be called at an intersection */
void DFS(int Vertex, int direction)
{
  //printf("\n%d",i);
  //DFS called upon new node, must move to node
  // robot do things

  //currently visiting a new node
  updateVertex();
  node *currNode;
  currNode=(node*)malloc(sizeof(node));
  currNode.vertex = currentVertex;
  currNode.x = xpos;
  currNode.y = ypos;
  currNode.walls = walls;
  currNode.back = back;
  addAllEdges(currNode);

  G[Vertex] = currNode;

  // Base case, while the adjacent neighbor of the current node is not NULL
  direction = 0;
  while(direction<4){
    Vertex = currNode.vertex;
    
    if(intersectFlag) {
      //idiot, you haven't visited anything yet   
      //choose new node to move to
      if ((G[Vertex].adjNodes[direction]>-1) && (G[G[Vertex].adjNodes[direction]]==NULL)){
        moveToNearbyNode(direction);
        DFS(Vertex, direction);
      }
      direction++;
    }
  }

  //backtrack to currNode first??
  //need return path damn...
  //calculate shortest path?  
  //visiting previous nodes now, not making new nodes 
  pathToNode(G[currNode.vertex]);
  
}
