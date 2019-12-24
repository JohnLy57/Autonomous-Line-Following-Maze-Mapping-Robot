// mux control pins
int mux_c = 3;
int mux_b = 4;
int mux_a = 7;

int mux_out = A1;

int y0,y1,y2,y3,y4,y5,y6,y7;

char buffer[60];


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(mux_c,OUTPUT);
  pinMode(mux_b,OUTPUT);
  pinMode(mux_a,OUTPUT);


}

void loop() {
  // put your main code here, to run repeatedly:
//
//  SENSOR LEFT
//    // SWEEP PINS:
   digitalWrite(mux_c, LOW);       
   digitalWrite(mux_b, LOW);       
   digitalWrite(mux_a, LOW);   
   y0 = analogRead(mux_out);
   delay(20);

//   digitalWrite(mux_c, HIGH);       
//   digitalWrite(mux_b, LOW);       
//   digitalWrite(mux_a, LOW);   
//   y1 = analogRead(mux_out);
//  delay(20);

   digitalWrite(mux_c, LOW);       
   digitalWrite(mux_b, HIGH);       
   digitalWrite(mux_a, LOW);   
//   y2 = analogRead(mux_out);
//   delay(20);

  //  digitalWrite(mux_c, LOW);       
  //  digitalWrite(mux_b, HIGH);       
  //  digitalWrite(mux_a, HIGH);   
  //  y3 = analogRead(A0);
  //  delay(20);


  //  digitalWrite(mux_c, HIGH);       
  //  digitalWrite(mux_b, LOW);       
  //  digitalWrite(mux_a, LOW);   
  //  y4 = analogRead(A0);
  //  delay(20);

  //  digitalWrite(mux_c, HIGH);       
  //  digitalWrite(mux_b, LOW);       
  //  digitalWrite(mux_a, HIGH);   
  //  y5 = analogRead(A0);
  //  delay(20);

  //  digitalWrite(mux_c, HIGH);       
  //  digitalWrite(mux_b, HIGH);       
  //  digitalWrite(mux_a, LOW);   
  //  y6 = analogRead(A0);
  //  delay(20);

  //   digitalWrite(mux_c, HIGH);       
  //   digitalWrite(mux_b, HIGH);       
  //   digitalWrite(mux_a, HIGH);   
  //   y7 = analogRead(A0);
  //   delay(20);

    //sprintf(buffer,"%d|%d|%d|%d|%d|%d|%d|%d",y0,y1,y2,y3,y4,y5,y6,y7);
    Serial.println(y0);



}
