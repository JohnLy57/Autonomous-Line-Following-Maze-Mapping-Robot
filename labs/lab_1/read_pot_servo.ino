/*
    Reads analog pin 0 and uses that value to set the speed and direction of a continuous servo motor
*/

#include <Servo.h>

int sensorPin = A0;    
int sensorValue = 0;  
Servo myservo;

void setup() {
  myservo.attach(10); //Initializes a servo on pin 10
}

void loop() {
  sensorValue = analogRead(sensorPin);              //Read analog pin 0 value
  int servoValue = map(sensorValue, 0,1023,0,180);  //Converts the adc value 0-1023 to a value 0-180
  myservo.write(servoValue);                        //Sets the speed and direction of the servo based on the potentiometer position
}