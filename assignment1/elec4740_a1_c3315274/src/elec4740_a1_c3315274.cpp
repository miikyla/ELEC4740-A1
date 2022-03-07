/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#line 1 "c:/Users/Mikyla/Desktop/ELEC4740/assignment1/elec4740_a1_c3315274/src/elec4740_a1_c3315274.ino"
/*
 * Project elec4740_a1_c3315274
 * Description: Assignment 1 for ELEC4740
 * Author: Mikyla Peters C3315274
 * Date: 07/03/2022
 */

#include "Particle.h"
#include "dct.h"
#include "HC-SR04.h"

/************************************
 *          INITIALISATION          *
 ************************************/
void setup();
void loop();
#line 15 "c:/Users/Mikyla/Desktop/ELEC4740/assignment1/elec4740_a1_c3315274/src/elec4740_a1_c3315274.ino"
SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(MANUAL);

// Movement sensor
const int triggerPin    = D1;
const int echoPin       = D0;
// HC_SR04 rangefinder     = HC_SR04(triggerPin, echoPin);

unsigned long start     = 0;
float cmDistance        = 0.0;
unsigned long calcTime  = 0;

// Light sensor
const int lightPin          = A1;

unsigned int pccResult      = 0;       // PCC "resistor" ADC value.
unsigned int resistorOne    = 10000;   // 10K resistor.
unsigned int resistorTwo    = 0;       // Converted resistor value from ADC value.

// Sound sensor
const int soundPin      = A2;

/************************************
 *          SETUP                   *
 ************************************/
void setup() {
    Serial.begin(9600);
    Particle.connect();

  // Initialise the light sensor


  // Initialise the sound sensor

  // Initialise the movement sensor
    pinMode(echoPin, INPUT); 
    pinMode(triggerPin, OUTPUT);
    //rangefinder.init();

}

/************************************
 *          LOOP                    *
 ************************************/
void loop() {

    // Test the light sensor
    // pccResult = analogRead(lightPin);

    // unsigned long lightRes = (resistorTwo/(resistorOne + resistorTwo)) * 3.3; // 5V power supply

    // Serial.printf("ADC value: %d\n", pccResult);

    // Test the sound sensor

    // Test the movement sensor
    digitalWrite(triggerPin, LOW); 
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH); // Pulse for 10μ s to trigger ultrasonic detection
    delayMicroseconds(10); 
    digitalWrite(triggerPin, LOW);
    int distance = pulseIn(echoPin, HIGH); // Read receiver pulse time 
    distance= distance/58; // Transform pulse time to distance 
    Serial.println(distance); //Output distance
    delay(50);
    // start = micros();
    // cmDistance = rangefinder.distCM();
    // calcTime = micros() - start;

    // Serial.printf("Range finding duration: %lu | Distance in cm: %.2f\n", calcTime, cmDistance);
    
    // delay(500);
}