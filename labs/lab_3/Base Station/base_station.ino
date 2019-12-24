int wall_d1 = 2;  //WEST
int wall_d2 = 3;  //SOUTH
int wall_d3 = 4;  //EAST
int wall_d4 = 5;  //NORTH
int xpos_d1 = 6;  
int xpos_d2 = 7;
int xpos_d3 = 8;
int xpos_d4 = 9;
int ypos_d1 = 10;
int ypos_d2 = 11;
int ypos_d3 = 12;
int ypos_d4 = 13;


int toggle = 0;

void setup() {
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);

  pinMode(1, INPUT);

}


void loop() {
  if(digitalRead(1)){
     if (!toggle){
      digitalWrite(2, HIGH);
      toggle = 1;
      delay(100);
     }
     else {
      digitalWrite(2,LOW);
      toggle = 0;
            delay(100);

     }
  } 
}
