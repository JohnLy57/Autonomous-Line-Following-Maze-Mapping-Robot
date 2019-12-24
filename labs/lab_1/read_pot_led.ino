/*
    Reads analog pin 0 and uses that value to set the brightness of an led on pin 9
*/

int sensorPin = A0;    
int sensorValue = 0;  
int ledPin = 9;

void setup() {
  pinMode(ledPin, OUTPUT);
}

void loop() {
  sensorValue = analogRead(sensorPin);  //Read analog pin 0 value
  analogWrite(ledPin,sensorValue/4);    //Converts the analog read value to a value from 0-255 to set the led brightness
}