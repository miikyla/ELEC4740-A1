/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#line 1 "c:/Users/Mikyla/Documents/GitHub/ELEC4740-A1/assignment1/elec4740_a1_c3315274/src/elec4740_a1_c3315274.ino"
/*
 * Project elec4740_a1_c3315274
 * Description: Assignment 1 for ELEC4740
 * Author: Mikyla Peters C3315274
 * Date: 07/03/2022
 */

#include "Particle.h"
#include "dct.h"
#include "math.h"

void setup();
void loop();
#line 12 "c:/Users/Mikyla/Documents/GitHub/ELEC4740-A1/assignment1/elec4740_a1_c3315274/src/elec4740_a1_c3315274.ino"
#define NUM_LIGHT_READS 100
#define NUM_SOUND_READS 500

/************************************
 *          INITIALISATION          *
 ************************************/
SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(MANUAL);

// Movement sensor
const int trigPin   = D0;
const int echoPin   = D1;
int duration        = 0;
int distance        = 0;

// Light sensor
const int lightPin          = A1;
const int rTwo              = 1000;

unsigned int lightRead      = 0;        // Averaged raw ADC read from the GPIO pin for the light sensor.
unsigned int lightResult    = 0;        // Transfer function from ADC to lux.
float lightVout             = 0;        // Light ADC reading converted to voltage.
unsigned long rPcc          = 0;        // Equivalent resistance of light sensor.
unsigned long lightLux[2]   = {0,0};    // Calcualted lux value, two measurements to check for hysteresis.

// Sound sensor
const int soundPin  = A2;

int soundPkPk       = 0;    // Pk-pk value of the incoming sound wave.
float soundVout     = 0;    // Sound ADC reading converted to voltage.
float soundDba      = 0;    // Calculated sound value in dBa.

/************************************
 *          SETUP                   *
 ************************************/
void setup() {
    Serial.begin(9600);
    Particle.connect();

    // Initialise the light sensor
    pinMode(lightPin, INPUT); 

    // Initialise the sound sensor
    pinMode(soundPin, INPUT); 

    // Initialise the movement sensor
    pinMode(echoPin, INPUT); 
    pinMode(trigPin, OUTPUT);

}

/************************************
 *          LOOP                    *
 ************************************/
void loop() {

    // ------------- LIGHT SENSOR -------------
    lightLux[1] = lightLux[0];
    unsigned int avgLightRead = 0;  // Summation of light reads.

    for (int i = 0; i < NUM_LIGHT_READS; i++)
    {
        avgLightRead += analogRead(lightPin);
    }

    lightRead = avgLightRead/NUM_LIGHT_READS;   // Read the ADC value for the light resistor.
    
    // Transfer function for the ADC reaidng to Vout
    lightVout = 3.3 * lightRead/4096;
    rPcc = (rTwo * 3.3) / (lightVout) - rTwo;

    // Calculate lux value with no offset applied.
    lightLux[0] = 5*pow(10,11) * pow(rPcc, -2.335);

    // Determine offset based on hysteresis of the light sensor.
    if (lightLux[0] >= lightLux[1]) // Light level is increasing.
    {
        if (lightLux[0] > 10) // Offset is only valid for lux values greater than 10.
        {
            lightLux[0] -= (4*pow(10,-5) * pow(lightLux[0],2) + 0.077 * lightLux[0] - 153.97);
        }
    }

    else // Light level is decreasing.
    {
        if (lightLux[0] > 10) // Offset is only valid for lux values greater than 10.
        {
            lightLux[0] -= pow(10,-4) * pow(lightLux[0],2) + 0.0407 * lightLux[0] - 104.35;
        }
        
    }

    Serial.printf("Light [lux]: ");
    Serial.println(lightLux[0]);

    // ------------- SOUND SENSOR -------------
    unsigned int tempSoundRead = 0;  // Summation of light reads.
    unsigned int maxAmpl = 0;
    unsigned int minAmpl = 4096;

    for(int i = 0; i < NUM_SOUND_READS; i++)
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
    //Serial.printf("2.1 ");
    //Serial.println(soundPkPk);

    soundVout = 3.3 * soundPkPk/4096;
    //Serial.printf("2.2 ");
    //Serial.println(soundVout);

    soundDba = 17.831 * log(soundVout) + 87.579;
    //Serial.printf("2.3 ");
    //Serial.println(soundDba);

    Serial.printf("------------------ Sound [dBa]: ");
    Serial.println(soundDba);


    // ------------- MOVEMENT SENSOR -------------

    // Clears the trigPin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);
    // Calculating the distance
    distance = duration * 0.034 / 2;
    
    // If the distance is greater than the sensors maximum allowable measurement, set value to zero.
    if(distance > 400)
    {
        distance = 0;
    }

    // Prints the distance on the Serial Monitor
    Serial.printf("------------------ ------------------ Distance [cm]: ");
    Serial.println(distance);
    
    delay(500);
}