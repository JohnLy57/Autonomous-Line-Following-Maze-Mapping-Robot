/*
    Blinks all digital pins (0-13) on the arduino
    i.e. toggle all outputs between 0-5V every 1 second
*/

void setup() {
  // initialize digital pin 0-13 as an output.
  for(int i = 0; i<=13; i++){
       pinMode(i, OUTPUT);
  }
}

void loop() {
  for(int i = 0; i<=13; i++){   //Turn on all digital pins
         digitalWrite(i, HIGH);  
  }
  delay(1000);   

  for(int i = 0; i<=13; i++){   //Turn off all digital pins
         digitalWrite(i, LOW);   
  }  
  delay(1000);                       
}