/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#line 1 "c:/Users/Mikyla/Desktop/ELEC4740/blinky/src/blinky.ino"
/*
* Project test
* Description: Blinking LED
* Author: Canvas
* Date: 28/02/2022
*/

#include "Particle.h"
#include "dct.h"
//SYSTEM_MODE(MANUAL);
void setup();
void loop();
#line 11 "c:/Users/Mikyla/Desktop/ELEC4740/blinky/src/blinky.ino"
SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(MANUAL);
int led1 = D7;
int led2 = D0;

// setup() runs once, when the device is first turned on.
void setup() {
  int i;
  const uint8_t val = 0x01;
  //dct_write_app_data(&val, DCT_SETUP_DONE_OFFSET, 1);
  // Put initialization like pinMode and begin functions here.
  //Particle.publish("hello","world",PRIVATE);
  pinMode(led1,OUTPUT);
  pinMode(led2,OUTPUT);

  for(i=0;i<8;i++)
  {
    // The core of your code will likely live here.
    digitalWrite(led1,HIGH);
    delay(200);
    digitalWrite(led1,LOW);
    delay(200);
  }

  // Do normal things here
  Particle.connect();
}

// loop() runs over and over again, as quickly as it can execute.
void loop() {
  // The core of your code will likely live here.
  digitalWrite(led1,HIGH);
  digitalWrite(led2,HIGH);
  delay(50);
  digitalWrite(led1,LOW);
  digitalWrite(led2,LOW);
  delay(1000);
}