/*
    Reads analog pin 0 and writes its converted adc value to the serial monitor
*/

int sensorPin = A0;    
int sensorValue = 0;  


void setup() {
  Serial.begin(9600);
}

void loop() {
  sensorValue = analogRead(sensorPin);  //Read the voltage on pin A0 (from the potentiometer voltage divider)
  Serial.println(sensorValue);          //Prints the ADC value (0-1023) that is corresponds to the analog voltages 0-5V
}