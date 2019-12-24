
/*
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>
 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

/**
 * Example for Getting Started with nRF24L01+ radios.
 *
 * This is an example of how to use the RF24 class.  Write this sketch to two
 * different nodes.  Put one of the nodes into 'transmit' mode by connecting
 * with the serial monitor and sending a 'T'.  The ping node sends the current
 * time to the pong node, which responds by sending the value back.  The ping
 * node can then see how long the whole cycle took.
 */

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

//
// Hardware configuration
//

// Set up nRF24L01 radio on SPI bus plus pins 9 & 10

RF24 radio(9,10);

typedef struct{
  bool visited;
  bool north;
  bool east;
  bool south;
  bool west;
} positions;

positions wall_array[9][9]; 

//
// Topology
//

// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = { 0x0000000076LL, 0x0000000077LL };


void exteriorWalls(void){
  unsigned int y = 0;
  for(unsigned int i = 0; i<9; i++){
    digitalWrite(A0, ((i)&1)); //Xpos, bit 0
    digitalWrite(A1, ((i>>1)&1));
    digitalWrite(A2, ((i>>2)&1));
    digitalWrite(A3, ((i>>3)&1));
    digitalWrite(A4, ((y)&1)); //Ypos, bit 0
    digitalWrite(A5, ((y>>1)&1));
    digitalWrite(2, ((y>>2)&1));
    digitalWrite(3, ((y>>3)&1));
    digitalWrite(4, LOW);
    digitalWrite(5, HIGH);
    digitalWrite(6, LOW);
    digitalWrite(7, LOW);
  
    digitalWrite(8, HIGH); //Toggle update
    digitalWrite(8, LOW);
  }
  y = 8;
  for(unsigned int i = 0; i<9; i++){
    digitalWrite(A0, ((i)&1)); //Xpos, bit 0
    digitalWrite(A1, ((i>>1)&1));
    digitalWrite(A2, ((i>>2)&1));
    digitalWrite(A3, ((i>>3)&1));
    digitalWrite(A4, ((y)&1)); //Ypos, bit 0
    digitalWrite(A5, ((y>>1)&1));
    digitalWrite(2, ((y>>2)&1));
    digitalWrite(3, ((y>>3)&1));
    digitalWrite(4, LOW);
    digitalWrite(5, LOW);
    digitalWrite(6, LOW);
    digitalWrite(7, HIGH);
  
    digitalWrite(8, HIGH); //Toggle update
    digitalWrite(8, LOW);
  }

  unsigned int x = 0;
  for(unsigned int i = 0; i<9; i++){
    digitalWrite(A0, ((x)&1)); //Xpos, bit 0
    digitalWrite(A1, ((x>>1)&1));
    digitalWrite(A2, ((x>>2)&1));
    digitalWrite(A3, ((x>>3)&1));
    digitalWrite(A4, ((i)&1)); //Ypos, bit 0
    digitalWrite(A5, ((i>>1)&1));
    digitalWrite(2, ((i>>2)&1));
    digitalWrite(3, ((i>>3)&1));
    digitalWrite(4, HIGH);
    digitalWrite(5, LOW);
    digitalWrite(6, LOW);
    digitalWrite(7, LOW);
  
    digitalWrite(8, HIGH); //Toggle update
    digitalWrite(8, LOW);
  }
  x = 8;
  for(unsigned int i = 0; i<9; i++){
    digitalWrite(A0, ((x)&1)); //Xpos, bit 0
    digitalWrite(A1, ((x>>1)&1));
    digitalWrite(A2, ((x>>2)&1));
    digitalWrite(A3, ((x>>3)&1));
    digitalWrite(A4, ((i)&1)); //Ypos, bit 0
    digitalWrite(A5, ((i>>1)&1));
    digitalWrite(2, ((i>>2)&1));
    digitalWrite(3, ((i>>3)&1));
    digitalWrite(4, LOW);
    digitalWrite(5, LOW);
    digitalWrite(6, HIGH);
    digitalWrite(7, LOW);
  
    digitalWrite(8, HIGH); //Toggle update
    digitalWrite(8, LOW);
  }
}

void setup(void)
{
  for(int i = 0; i<9; i++){
    for(int j = 0; j<9; j++){
      wall_array[i][j] = {false,false,false,false,false};
    }
  }
  
  pinMode(A0, OUTPUT); //Xpos, bit 0
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A4, OUTPUT); //Ypos, bit 0
  pinMode(A5, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT); //West
  pinMode(5, OUTPUT); //South
  pinMode(6, OUTPUT); //East
  pinMode(7, OUTPUT); //North
  pinMode(8, OUTPUT); //Update Bit
  //
  // Print preamble
  //

  //Serial.begin(9600);
  //printf_begin();

  //
  // Setup and configure rf radio
  //

  radio.begin();

  // optionally, increase the delay between retries & # of retries
  radio.setRetries(15,15);
  radio.setAutoAck(true);
  // set the channel
  radio.setChannel(0x50);
  // set the power
  // RF24_PA_MIN=-18dBm, RF24_PA_LOW=-12dBm, RF24_PA_HIGH=-6dBM, and RF24_PA_MAX=0dBm.
  radio.setPALevel(RF24_PA_MAX);
  //RF24_250KBPS for 250kbs, RF24_1MBPS for 1Mbps, or RF24_2MBPS for 2Mbps
  radio.setDataRate(RF24_250KBPS);

  // optionally, reduce the payload size.  seems to
  // improve reliability
  //radio.setPayloadSize(8);

  //
  // Open pipes to other nodes for communication
  //

  //Receiver
  radio.openWritingPipe(pipes[1]);
  radio.openReadingPipe(1,pipes[0]);

  //
  // Start listening
  //

  radio.startListening();

  //
  // Dump the configuration of the rf unit for debugging
  //

  //radio.printDetails();
  exteriorWalls();
}


void loop(void)
{
  //
  // Pong back role.  Receive each packet, dump it out, and send it back
  //

  //radio.startListening();
  
  // if there is data ready
  if ( radio.available() )
  {
    // Dump the payloads until we've gotten everything
    unsigned int data = 0;
    int xpos = -1;
    int ypos = -1;
    int wall = -1;
    bool done = false;
    while (!done)
    {
      // Fetch the payload, and see if this was the last one.
      done = radio.read( &data, sizeof(unsigned int) );
      
    }
    //Serial.println(data);
    xpos = data&15; 
    //Serial.println(xpos);
    ypos = ((unsigned int)data>>4)&15;
    //Serial.println(ypos);
    wall = ((unsigned int)data>>8)&15;
    //Serial.println(wall);
    //Serial.println();

    if(!wall_array[xpos][ypos].visited){
      wall_array[xpos][ypos] = {true, (bool)((wall>>3)&1),(bool)((wall>>2)&1),(bool)((wall>>1)&1),(bool)((wall)&1)};
    }
    //Serial.println(walls[xpos][ypos].north);
    //Serial.println(walls[xpos][ypos].east);
    //Serial.println(walls[xpos][ypos].south);
    //Serial.println(walls[xpos][ypos].west);

    digitalWrite(A0, ((xpos)&1)); //Xpos, bit 0
    digitalWrite(A1, ((xpos>>1)&1));
    digitalWrite(A2, ((xpos>>2)&1));
    digitalWrite(A3, ((xpos>>3)&1));
    digitalWrite(A4, ((ypos)&1)); //Ypos, bit 0
    digitalWrite(A5, ((ypos>>1)&1));
    digitalWrite(2, ((ypos>>2)&1));
    digitalWrite(3, ((ypos>>3)&1));

    if(wall&1){ //West
      digitalWrite(4, HIGH);
      digitalWrite(5, LOW);
      digitalWrite(6, LOW);
      digitalWrite(7, LOW);

      digitalWrite(8, HIGH); //Toggle update
      digitalWrite(8, LOW);
    }

    if((wall>>1)&1){ //South
      digitalWrite(4, LOW);
      digitalWrite(5, HIGH);
      digitalWrite(6, LOW);
      digitalWrite(7, LOW); 

      digitalWrite(8, HIGH); //Toggle update
      digitalWrite(8, LOW);
    }

    if((wall>>2)&1){ //East
      digitalWrite(4, LOW);
      digitalWrite(5, LOW);
      digitalWrite(6, HIGH);
      digitalWrite(7, LOW); 

      digitalWrite(8, HIGH); //Toggle update
      digitalWrite(8, LOW);
    }

    if((wall>>3)&1){ //North
      digitalWrite(4, LOW);
      digitalWrite(5, LOW);
      digitalWrite(6, LOW);
      digitalWrite(7, HIGH); 

      digitalWrite(8, HIGH); //Toggle update
      digitalWrite(8, LOW);
    }
    
    
    //digitalWrite(A0, (xpos>>1)&1);
  }
}

