/*
fft_adc.pde
guest openmusiclabs.com 8.18.12
example sketch for testing the fft library.
it takes in data on ADC0 (Analog0) and processes them
with the fft. the data is sent out over the serial
port at 115.2kb.  there is a pure data patch for
visualizing the data.
*/

#define LOG_OUT 1 // use the log output function
#define FFT_N 256 // set to 256 point fft

#include <FFT.h> // include the library

void setup() {
  Serial.begin(9600); // use the serial port
  TIMSK0 = 0; // turn off timer0 for lower jitter
  ADCSRA = 0xe7; // changed prescaler from 32 to 128
  ADMUX = 0x40; // use adc0
  DIDR0 = 0x01; // turn off the digital input for adc0
  pinMode(7,OUTPUT);
  pinMode(13,OUTPUT);

}

void loop() {
  while(1) { // reduces jitter
    cli();  // UDRE interrupt slows this way down on arduino1.0
    for (int i = 0 ; i < 512 ; i += 2) { // save 256 samples
      while(!(ADCSRA & 0x10)); // wait for adc to be ready
      ADCSRA = 0xf5; // restart adc
      byte m = ADCL; // fetch adc data
      byte j = ADCH;
      int k = (j << 8) | m; // form into an int
      k -= 0x0200; // form into a signed int
      k <<= 6; // form into a 16b signed int
      fft_input[i] = k; // put real data into even bins
      fft_input[i+1] = 0; // set odd bins to 0
    }
    fft_window(); // window the data for better frequency response
    fft_reorder(); // reorder the data before doing the fft
    fft_run(); // process the data in the fft
    fft_mag_log(); // take the output of the fft
    sei();
//    Serial.write(255); // send a start byte
//    Serial.write(fft_log_out, 128); // send out the data
    Serial.println("FFT Begin");
    //String out = "";
    //for (byte i = 0 ; i < 128; i++) {
//      Serial.println(out + fft_log_out[i] + " Bin:" +i); //send out data to serial
    //}
    if (fft_log_out[6] > 120){ //Turn LED on when bin 6 has a high enough magnitude - about 950Hz
      digitalWrite(7,HIGH);
      digitalWrite(13,HIGH);
    }
    else {
      digitalWrite(7,LOW);
      digitalWrite(13,LOW);
    }
  }
}
