int mux_a = 7;
int mux_b = 8;
int mux_c = 9;

int irFront = 0;
int irBack = 0;
int irLeft = 0;
int irRight = 0;


void setup() {
  Serial.begin(9600);
  pinMode(mux_a, OUTPUT);
  pinMode(mux_b, OUTPUT);
  pinMode(mux_c, OUTPUT);
}

void loop() {
  digitalWrite(mux_a, LOW);       
  digitalWrite(mux_b, LOW);       
  digitalWrite(mux_c, HIGH);
  irFront = analogRead(A3); // 4
  digitalWrite(mux_a, LOW);       
  digitalWrite(mux_b, HIGH);       
  digitalWrite(mux_c, HIGH);
  irRight = analogRead(A3); // 6
  digitalWrite(mux_a, HIGH);       
  digitalWrite(mux_b, HIGH);       
  digitalWrite(mux_c, HIGH);
  irLeft = analogRead(A3); // 7
  digitalWrite(mux_a, HIGH);       
  digitalWrite(mux_b, LOW);       
  digitalWrite(mux_c, HIGH);
  irBack = analogRead(A3); // 5

  Serial.print("irFront = "); 
  Serial.print(irFront);
  Serial.print(" | ");
  Serial.print("irBack = "); 
  Serial.print(irBack);
  Serial.print(" | ");
  Serial.print("irleft = "); 
  Serial.print(irLeft);
  Serial.print(" | ");
  Serial.print("irRight = "); 
  Serial.println(irRight);
}
