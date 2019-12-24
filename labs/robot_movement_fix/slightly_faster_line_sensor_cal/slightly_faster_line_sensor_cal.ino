int lineSensorL = A3;
int lineSensorM = A5;
int lineSensorR = A4;
long lsl_bright;
long lsl_dark;
long lsl_av;
long lsc_bright;
long lsc_dark;
long lsc_av;
long lsr_bright;
long lsr_dark;
long lsr_av;

int sample = 75;

void buff_clear(){
  while(Serial.available() != 0){
    Serial.read();
  }
}

void setup(){
  Serial.begin(9600);
}

void loop() {
  int state_flag = 0;

  //while(1){
  //  int shit = analogRead(lineSensorM);
  //      Serial.print("\n");
  //  Serial.print(shit);
  //}
  
  Serial.print("Place LEFT line sensor on DARK surface, i.e. ground. Press 1 when ready.\n");
  while(state_flag != 1){
    while(Serial.available() == 0) { }
    if (Serial.available() > 0){
      int readByte = Serial.parseInt();
      if (readByte ==  1){
        for(int x =0; x<sample; x++){
          lsl_dark += analogRead(lineSensorL);
        }
        lsl_dark = lsl_dark/sample;
        state_flag = 1;
        Serial.print("Calibration done.\n");
      }
      else{
        Serial.print("1 not pressed. Try again.\n");
      }
      buff_clear();
    }
  }
  
  delay(100);
  state_flag = 0;
  buff_clear();

  Serial.print("Place CENTER line sensor on DARK surface, i.e. ground. Press 1 when ready.\n");
  while(state_flag != 1){
    while(Serial.available() == 0) { }
    if (Serial.available() > 0){
      int readByte = Serial.parseInt();
      if (readByte ==  1){
        for(int x =0; x<sample; x++){
          lsc_dark += analogRead(lineSensorM);
        }
        lsc_dark = lsc_dark/sample;
        state_flag = 1;
        Serial.print("Calibration done.\n");
      }
      else{
        Serial.print("1 not pressed. Try again.\n");
      }
      buff_clear();
    }
  }
  
  delay(100);
  state_flag = 0;
  buff_clear();

  Serial.print("Place RIGHT line sensor on DARK surface, i.e. ground. Press 1 when ready.\n");
  while(state_flag != 1){
    while(Serial.available() == 0) { }
    if (Serial.available() > 0){
      int readByte = Serial.parseInt();
      if (readByte ==  1){
        for(int x =0; x<sample; x++){
          lsr_dark += analogRead(lineSensorR);
        }
        lsr_dark = lsr_dark/sample;
        state_flag = 1;
        Serial.print("Calibration done.\n");
      }
      else{
        Serial.print("1 not pressed. Try again.\n");
      }
      buff_clear();
    }
  }
  
  delay(100);
  state_flag = 0;
  buff_clear();
  
  Serial.print("Place LEFT line sensor on BRIGHT surface, i.e. ground. Press 2 when ready.\n");
  while(state_flag != 1){
    while(Serial.available() == 0) { }
    if (Serial.available() > 0){
      int readByte = Serial.parseInt();
      if (readByte ==  2){
        for(int x =0; x<sample; x++){
          lsl_bright += analogRead(lineSensorL);
        }
        lsl_bright = lsl_bright/sample;
        state_flag = 1;
        Serial.print("Calibration done.\n");

      }
      else{
        Serial.print("2 not pressed. Try again.\n");
      }
      buff_clear();
    }
  }

  delay(100);
  state_flag = 0;
  buff_clear();
  
  Serial.print("Place CENTER line sensor on BRIGHT surface, i.e. ground. Press 2 when ready.\n");
  while(state_flag != 1){
    while(Serial.available() == 0) { }
    if (Serial.available() > 0){
      int readByte = Serial.parseInt();
      if (readByte ==  2){
        for(int x =0; x<sample; x++){
          lsc_bright += analogRead(lineSensorM);
        }
        lsc_bright = lsc_bright/sample;
        state_flag = 1;
        Serial.print("Calibration done.\n");

      }
      else{
        Serial.print("2 not pressed. Try again.\n");
      }
      buff_clear();
    }
  }

  delay(100);
  state_flag = 0;
  buff_clear();
  
  Serial.print("Place RIGHT line sensor on BRIGHT surface, i.e. ground. Press 2 when ready.\n");
  while(state_flag != 1){
    while(Serial.available() == 0) { }
    if (Serial.available() > 0){
      int readByte = Serial.parseInt();
      if (readByte ==  2){
        for(int x =0; x<sample; x++){
          lsr_bright += analogRead(lineSensorR);
        }
        lsr_bright = lsr_bright/sample;
        state_flag = 1;
        Serial.print("Calibration done.\n");

      }
      else{
        Serial.print("2 not pressed. Try again.\n");
      }
      buff_clear();
    }
  }
  
  delay(100);
  state_flag = 0;
  buff_clear();

  lsl_av = (lsl_bright + lsl_dark)/2;
  lsc_av = (lsc_bright + lsc_dark)/2;
  lsr_av = (lsr_bright + lsr_dark)/2;
  int total_av = (lsl_av+lsc_av+lsr_av)/3;
  Serial.print("\n----------------------\n");
  Serial.print("COMPLETED CALIBRATION:\n");
  Serial.print("----------------------\n");

  Serial.print("LEFT BRIGHT: ");
  Serial.print(lsl_bright, DEC);
  Serial.print("\n");
  Serial.print("LEFT DARK: ");
  Serial.print(lsl_dark, DEC);
  Serial.print("\n");
  Serial.print("CENTER BRIGHT: ");
  Serial.print(lsc_bright, DEC);
  Serial.print("\n");
  Serial.print("CENTER DARK: ");
  Serial.print(lsc_dark, DEC);
  Serial.print("\n");
  Serial.print("RIGHT BRIGHT: ");
  Serial.print(lsr_bright, DEC);
  Serial.print("\n");
  Serial.print("RIGHT DARK: ");
  Serial.print(lsr_dark, DEC);
  Serial.print("\n");

  Serial.print("\n--------\n");
  Serial.print("THRESHOLD:\n");
  Serial.print("----------\n");

  Serial.print("LEFT MIDPOINT: ");
  Serial.print(lsl_av, DEC);
  Serial.print("\n");
  
  Serial.print("MIDDLE MIDPOINT: ");
  Serial.print(lsc_av, DEC);
  Serial.print("\n");

  Serial.print("RIGHT MIDPOINT: ");
  Serial.print(lsr_av, DEC);
  Serial.print("\n");
  
  Serial.print("GENERAL: MIDPOINT: ");
  Serial.print(total_av, DEC);
  Serial.print("\n");
  while(1);
}
