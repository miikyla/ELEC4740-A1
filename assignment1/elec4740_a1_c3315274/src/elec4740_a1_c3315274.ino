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
SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(MANUAL);

// Movement sensor
const int triggerPin    = D1;
const int echoPin       = D0;

unsigned long start     = 0;
float cmDistance        = 0.0;
unsigned long calcTime  = 0;

// Light sensor
const int lightPin          = A1;

unsigned int lightRead      = 0;    // Averaged raw ADC read from the GPIO pin for the light sensor.
unsigned long lightResult   = 0;    // Transfer function from ADC to lux.
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