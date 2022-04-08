/*
 * Project elec4740_a1_c3315274
 * Description: Assignment 1 for ELEC4740
 * Author: Mikyla Peters C3315274
 * Date: 07/03/2022
 */

#include "Particle.h"
#include "dct.h"
#include "HC-SR04.h"
#include "math.h"

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
unsigned int lightResult   = 0;    // Transfer function from ADC to lux.
float lightVout     = 0;
float lux           = 0;
float rPcc          = 0;
long lightLux      = 0;

// Sound sensor
const int soundPin      = A2;
int soundPkPk           = 0;

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
    unsigned int avgLightRead = 0;  // Summation of light reads.

    for(int i = 0; i < 100; i++)
    {
        avgLightRead += analogRead(lightPin);
    }

    lightRead = avgLightRead/100;               // Read the ADC value for the light resistor.
    
    // Transfer function for the ADC reaidng to Vout
    lightVout = 3.3 * lightRead/4096;
    rPcc = (1000 * 3.3) / (lightVout) - 1000;
    lightLux = 9512 * exp(-0.939 * rPcc);

   // Serial.printf("ADC: %d | vout: %f | R2: %f | lux: %f \n", lightRead, lightVout, rPcc, lightLux);

    // Test the sound sensor
    unsigned int tempSoundRead = 0;  // Summation of light reads.
    unsigned int maxAmpl = 0;
    unsigned int minAmpl = 4096;

    for(int i = 0; i < 500; i++)
    {
        tempSoundRead = analogRead(soundPin);

        if (tempSoundRead > maxAmpl)
        {
            maxAmpl = tempSoundRead;
        }

        if (tempSoundRead < minAmpl)
        {
            minAmpl = tempSoundRead;
        }
    }

    soundPkPk = maxAmpl - minAmpl;
    Serial.printf("max: %d | min: %d | pk: %d \n", maxAmpl, minAmpl, soundPkPk);


    // Test the movement sensor
    // digitalWrite(triggerPin, LOW); 
    // delayMicroseconds(2);
    // digitalWrite(triggerPin, HIGH); // Pulse for 10μ s to trigger ultrasonic detection
    // delayMicroseconds(10); 
    // digitalWrite(triggerPin, LOW);
    // int distance = pulseIn(echoPin, HIGH); // Read receiver pulse time 
    // distance= distance/58; // Transform pulse time to distance 
    // Serial.println(distance); //Output distance
    // delay(50);
    // start = micros();
    // cmDistance = rangefinder.distCM();
    // calcTime = micros() - start;

    // Serial.printf("Range finding duration: %lu | Distance in cm: %.2f\n", calcTime, cmDistance);
    
    delay(500);
}