/*
    Blinks the led on pin 0 every 1 second
*/

void setup() {
  pinMode(0, OUTPUT);     //LED is connected to pin 0.
}

void loop() {
  digitalWrite(0, HIGH);  //Turn pin 0 on (5V) 
  delay(1000);            //Wait 1s
  digitalWrite(0, LOW);   //Turn pin 0 off 
  delay(1000);            //Wait 1s 
}