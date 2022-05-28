#define SN1
//#define SN2
//#define CENTRAL

#include "Particle.h"
#include "math.h"
#include "DFRobot_LCD.h"

SYSTEM_MODE(MANUAL);

#define MS_IN_HR 3600000
#define MS_IN_10_MIN 600000

#ifdef SN1

#define NUM_LIGHT_READS 100
#define NUM_SOUND_READS 500

#define LIGHT_50 128
#define LIGHT_75 191
#define LIGHT_100 255

enum lightLevel
{
    L_OFF,
    L_LOW,
    L_MED,
    L_HIGH
};

enum alarmSource
{
    NONE,
    MOVEMENT,
    SOUND
};

// ----------------------------------------------------------------------------
// SYSTEM VARIABLES
// ----------------------------------------------------------------------------
// System state
bool b_is_security_mode     = 0;    // Set high when the sensor node is operating in SECURITY mode.
bool b_is_security_event    = 0;    // Set high when a notifiable security event has occured.
bool b_in_power_budget      = 1;    // Set low when the hourly power consumption budget has been exceeded.
alarmSource eventTrigger    = NONE; // Cause of security incident when operating in SECURITY mode.

long curr_light_val         = 0;
lightLevel prev_light_lvl   = L_OFF;
lightLevel curr_light_lvl   = L_OFF;

uint16_t light_pwr_consumption[4]   = {0, 5, 7, 10};    // Power consumed by the light module when in each enum lightLevel [mW]
unsigned long light_pwr_time[2]     = {0,0};            // Start time and end time of the current light level to calculate power consumption [ms]
float total_pwr_consumption         = 0;                // Lifetime power consumption of the device since bootup [mWs]
const float pwr_budget              = 2100*3600;        // Maximum amount of power consumption allowed per hour before power management kicks and throttles NORMAL mode [mWs] (NOTE: in seconds since float becomes too small).

float curr_sound_val        = 0;
uint8_t sound_duration_sec  = 0;
bool b_is_first_sound_instance  = 0;
unsigned long sound_start_time  = 0;

int curr_dist_val           = 0;
int prev_dist_val           = 0; 
bool b_is_first_dist_read   = 1;

unsigned long time_since_last_msg   = 0; // Holds the last time that a Bluetooth message was sent to the Cluster Head.

// Lamp actuator
const int lampPin   = D2;

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

// ----------------------------------------------------------------------------
// BLUETOOTH
// ----------------------------------------------------------------------------

// UUIDs for service + characteristics
const char* serviceUuid = "b4250400-fb4b-4746-b2b0-93f0e61122c6";
const char* light       = "b4250401-fb4b-4746-b2b0-93f0e61122c6";
const char* sound       = "b4250402-fb4b-4746-b2b0-93f0e61122c6";
const char* movement    = "b4250403-fb4b-4746-b2b0-93f0e61122c6";
const char* security    = "b4250404-fb4b-4746-b2b0-93f0e61122c6";

// Set up characteristics
BleUuid sensorNodeService(serviceUuid);

// Static function for handling Bluetooth Low Energy callbacks
static void onDataReceived(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context) {
    // If the control node has changed state, update the functionality of the sensor node.
    if(context == security) 
    {
        b_is_security_mode = data[0];
    }
}


BleCharacteristic lightCharacteristic("light", BleCharacteristicProperty::NOTIFY, light, serviceUuid, onDataReceived, (void*)light);
BleCharacteristic soundCharacteristic("sound", BleCharacteristicProperty::NOTIFY, sound, serviceUuid, onDataReceived, (void*)sound);
BleCharacteristic movementCharacteristic("movement", BleCharacteristicProperty::NOTIFY, movement, serviceUuid, onDataReceived, (void*)movement);
BleCharacteristic securityCharacteristic("security", BleCharacteristicProperty::WRITE, security, serviceUuid, onDataReceived, (void*)security);

// ----------------------------------------------------------------------------
// SLEEP MODE
// ----------------------------------------------------------------------------
SystemSleepConfiguration sleep_config;

// ----------------------------------------------------------------------------
// SETUP
// ----------------------------------------------------------------------------
void setup() {
    // Add the Bluetooth characteristics.
    BLE.addCharacteristic(lightCharacteristic);
    BLE.addCharacteristic(soundCharacteristic);
    BLE.addCharacteristic(movementCharacteristic);
    BLE.addCharacteristic(securityCharacteristic);

    // Begin advertising
    BleAdvertisingData advData;
    advData.appendServiceUUID(sensorNodeService);
    BLE.advertise(&advData);

    // Initialise the sensor pins
    pinMode(lampPin, OUTPUT);

    pinMode(lightPin, INPUT); 

    pinMode(soundPin, INPUT); 

    pinMode(echoPin, INPUT); 
    pinMode(trigPin, OUTPUT);

    // Initialise sleep mode characteristics. Only able to wake from a change in the light levels or a BLE message, or if 60 minutes has passed.
    sleep_config.mode(SystemSleepMode::ULTRA_LOW_POWER).duration(60min).ble().analog(lightPin, 1000, AnalogInterruptMode::CROSS);

    // Sets the RGB LED control so we can see what state the sensor node is in.
    RGB.control(true);

    curr_light_val = analogRead(lightPin);
}

// ----------------------------------------------------------------------------
// LOOP
// ----------------------------------------------------------------------------
void loop() {

    // Calculate how many hours board has been on
    uint16_t hours_up = floor(millis()/MS_IN_HR) + 1; // Ceil not working properly?

    // Check if the power budget for the hour has been exceeded.
    b_in_power_budget = (total_pwr_consumption > hours_up*pwr_budget) ? 0 : 1;

    // Running in NORMAL mode
    if(!b_is_security_mode && b_in_power_budget)
    {
        // Set the LED to green.
        RGB.color(0x00,0xFF,0x00);

        // If it's been more than 10 minutes since a status update has been sent to the Cluster Head, send one.
        if((millis() - time_since_last_msg) > MS_IN_10_MIN)
        {
            // Send the updated information to the cluster head.
            // D[0] - D[1]:     Light value         [lux]
            // D[2] - D[3]:     Power consumption   [mW]
            uint32_t lightData = 0;

            lightData |= curr_light_val & 0xFF;
            lightData |= light_pwr_consumption[curr_light_lvl] << 16;

            lightCharacteristic.setValue(lightData);
            time_since_last_msg = millis();
        }

        // Determine the intensity that the LED needs to be.
        curr_light_val = readLightLevel();

        if(curr_light_val > 400)
        {
            curr_light_lvl = L_LOW;
        }

        else if((curr_light_val > 200) && (curr_light_val < 400))
        {
            curr_light_lvl = L_MED;
        }

        else if(curr_light_val < 200)
        {
            curr_light_lvl = L_HIGH;
        }

        // If there has been a change in LED state, update the CH.
        if (curr_light_lvl != prev_light_lvl)
        {
            // Calculate the total amount of time that the module has been running at the (now previous) light intensity level.
            light_pwr_time[1] = light_pwr_time[0];
            light_pwr_time[0] = millis(); // Used to calculate the amount of power consumed.
            float time_spent_sec = (light_pwr_time[0] - light_pwr_time[1]) * 1/1000; // ms * sec/ms

            // Increment the total power consumption.
            total_pwr_consumption += light_pwr_consumption[prev_light_lvl] * time_spent_sec;

            // Change the light level.
            switch(curr_light_lvl)
            {
                case L_LOW:
                {
                    analogWrite(lampPin, LIGHT_50);
                    break;
                }

                case L_MED:
                {
                    analogWrite(lampPin, LIGHT_75);
                    break;
                }

                case L_HIGH:
                {
                    analogWrite(lampPin, LIGHT_100);
                    break;
                }

                default:
                {
                    break;
                }
            }

            // Send the updated information to the cluster head.
            // D[0] - D[1]:     Light value         [lux]
            // D[2] - D[3]:     Power consumption   [mW]
            uint32_t lightData = 0;
            Serial.printf("CURRENT LIGHT LEVEL IS: %d", curr_light_val);

            lightData |= (uint16_t)curr_light_val;
            lightData |= light_pwr_consumption[curr_light_lvl] << 16;

            lightCharacteristic.setValue(lightData);
            time_since_last_msg = millis();

            // Current state is now the previous state since the CH has been updated.
            prev_light_lvl = curr_light_lvl;
        }
    }

    // Running in throttled NORMAL mode
    else if(!b_is_security_mode && !b_in_power_budget)
    {
        // Set the LED to red.
        RGB.color(0xFF,0x00,0x00);

        // In throttled mode, light level thresholds and the resulting light intensity have been altered to preserve battery life.
        // Additionally, if the light level is high enough, the node will go into sleep mode. It will only be able to be woken if the light ADC value cross the 400 lux threshold or if the Cluster Head sends data.

        // Determine the intensity that the LED needs to be.
        curr_light_val = readLightLevel();

        if(curr_light_val > 400)
        {
            curr_light_lvl = L_OFF;
        }

        else{
            curr_light_lvl = L_LOW;
        }

        // If there has been a change in LED state, update the CH.
        if (curr_light_lvl != prev_light_lvl)
        {
            // Calculate the total amount of time that the module has been running at the (now previous) light intensity level.
            light_pwr_time[1] = light_pwr_time[0];
            light_pwr_time[0] = millis(); // Used to calculate the amount of power consumed.
            float time_spent_hr = (light_pwr_time[0] - light_pwr_time[1]) * 1/1000 * 1/60 * 1/60; // ms * sec/ms * min/sec * hour/min

            // Increment the total power consumption.
            total_pwr_consumption += light_pwr_consumption[prev_light_lvl] * time_spent_hr;

            // Change the light level.
            switch(curr_light_lvl)
            {
                case L_OFF:
                {
                    analogWrite(lampPin, 0);
                    break;
                }

                case L_LOW:
                {
                    analogWrite(lampPin, LIGHT_50);
                    break;
                }

                default:
                {
                    break;
                }
            }

            // Send the updated information to the cluster head.
            // D[0] - D[1]:     Light value         [lux]
            // D[2] - D[3]:     Power consumption   [mW]
            uint32_t lightData = 0;

            lightData |= curr_light_val & 0xFF;
            lightData |= light_pwr_consumption[curr_light_lvl] << 16;

            lightCharacteristic.setValue(lightData);
            time_since_last_msg = millis();

            // Current state is now the previous state since the CH has been updated.
            prev_light_lvl = curr_light_lvl;
        }

        // If the light is off, go into sleep mode.
        if(prev_light_lvl == L_OFF)
        {
            System.sleep(sleep_config);
        }

    }

    // Running in SECURITY mode
    else
    {
        // Turn off the actuators.
        curr_light_lvl = L_OFF;
        analogWrite(lampPin, 0);

        // Set the LED to blue.
        RGB.color(0x00,0x00,0xFF);

        // Read the movement sensor

        // To prevent a false positive security event, make both values equal to the same value on the first read.
        if(b_is_first_dist_read)
        {
            b_is_first_dist_read = 0;
            curr_dist_val = readDistance();
            prev_dist_val = curr_dist_val;
        }

        prev_dist_val = curr_dist_val;
        curr_dist_val = readDistance();

        // If the distance change was greater than 10cm, can assume that movement has occured.
        if(abs(curr_dist_val - prev_dist_val) > 10)
        {
            b_is_security_event = 1;
            eventTrigger = MOVEMENT;
        }

        // Read the sound sensor.
        curr_sound_val = readSoundLevel();

        if(curr_sound_val > 80)
        {
            b_is_security_event = 1;
            eventTrigger = SOUND;
        } 
        else if(curr_sound_val > 70)
        {
            // If this is the first time that it's been this sound level, begin counting the duration.
            if(!b_is_first_sound_instance)
            {
                b_is_first_sound_instance = 1;
                sound_start_time = millis();
            }
            
            else
            {
                unsigned long current_sound_time = millis();

                // If it's been longer than 10 seconds, report a security event.
                if((current_sound_time - sound_start_time) > 10000)
                {
                    sound_duration_sec = (uint8_t)((current_sound_time - sound_start_time)/1000);
                    b_is_security_event = 1;
                    b_is_first_sound_instance = 0;
                }
            }

        }
        else if((curr_sound_val <= 70) && (curr_sound_val >= 55))
        {
            // If this is the first time that it's been this sound level, begin counting the duration.
            if(!b_is_first_sound_instance)
            {
                b_is_first_sound_instance = 1;
                sound_start_time = millis();
            }
            
            else
            {
                unsigned long current_sound_time = millis();

                // If it's been longer than 30 seconds, report a security event.
                if((current_sound_time - sound_start_time) > 30000)
                {
                    sound_duration_sec = (uint8_t)((current_sound_time - sound_start_time)/1000);
                    b_is_security_event = 1;
                    b_is_first_sound_instance = 0;
                }
            }
        }

        // If a security event has been detected, notify the CH.
        if(b_is_security_event)
        {
            // Send the updated information to the cluster head.
            switch(eventTrigger)
            {
                // -- Sound event --
                // D[0]: Sound level [dbA]
                // D[1]: Sound duration [sec]
                case SOUND:
                {
                    uint16_t soundData = 0;
                    soundData |= (uint8_t)curr_sound_val;
                    soundData |= sound_duration_sec << 8;

                    soundCharacteristic.setValue(soundData);
                    time_since_last_msg = millis();
                    break;
                }

                // -- Movement event --
                // D[0] - D[1]: Distance [cm]
                case MOVEMENT:
                {
                    uint16_t movementData = (uint16_t)curr_dist_val;
                    Serial.printf("THE current distance value is %d", movementData);

                    movementCharacteristic.setValue(movementData);
                    time_since_last_msg = millis();
                    break;
                }
                
                default: // Also covers NONE.
                {
                    break;
                }
            }

            // Reset the trigger now that the CH has been notified.
            eventTrigger = NONE;
            b_is_security_event = 0;
        }
    }
}

/*
 * @brief Reads the current light level from the sensor.
 * @returns The resulting light level in lux.
*/
long readLightLevel()
{
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

    return lightLux[0];
}

/*
 * @brief Reads the current sound level from the sensor.
 * @returns The resulting sound level in dBa.
*/
float readSoundLevel()
{
    unsigned int tempSoundRead = 0;  // Summation of sound reads.
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
    soundVout = 3.3 * soundPkPk/4096;
    soundDba = 17.831 * log(soundVout) + 87.579;  

    return 10; // FIX ME
}

/*
 * @brief Reads the distance of an object from the sensor.
 * @returns The resulting distance in cm.
*/
int readDistance()
{
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

    return distance;
}

#endif

#ifdef SN2

#define NUM_SOUND_READS 500
#define NUM_TEMP_READS 10

#define FAN_OFF 0
#define FAN_1 100
#define FAN_2 255

enum fanLevel
{
    F_OFF,
    F_LOW,
    F_HIGH
};

enum alarmSource
{
    NONE,
    MOVEMENT,
    SOUND
};

// ----------------------------------------------------------------------------
// SYSTEM VARIABLES
// ----------------------------------------------------------------------------
// System state
bool b_is_security_mode     = 0;    // Set high when the sensor node is operating in SECURITY mode.
bool b_is_security_event    = 0;    // Set high when a notifiable security event has occured.
bool b_in_power_budget      = 1;    // Set low when the hourly power consumption budget has been exceeded.
alarmSource eventTrigger    = NONE; // Cause of security incident when operating in SECURITY mode.

long curr_temp_val        = 0;
fanLevel prev_temp_lvl    = F_OFF;
fanLevel curr_temp_lvl    = F_OFF;

uint16_t fan_pwr_consumption[4]   = {0, 392, 1000};     // Power consumed by the fan module when in each enum fanLevel [mW]
unsigned long fan_pwr_time[2]     = {0,0};              // Start time and end time of the current fan level to calculate power consumption [ms]
float total_pwr_consumption         = 0;                // Lifetime power consumption of the device since bootup [mWs]
const float pwr_budget              = 2100*3600;        // Maximum amount of power consumption allowed per hour before power management kicks and throttles NORMAL mode [mWs] (NOTE: in seconds since float becomes too small).

float curr_sound_val        = 0;
uint8_t sound_duration_sec  = 0;
bool b_is_first_sound_instance  = 0;
unsigned long sound_start_time  = 0;

int curr_dist_val           = 0;
int prev_dist_val           = 0; 
bool b_is_first_dist_read   = 1;

unsigned long time_since_last_msg   = 0; // Holds the last time that a Bluetooth message was sent to the Cluster Head.

// Fan actuator
const int fanPin    = D3;

// Movement sensor
const int trigPin   = D0;
const int echoPin   = D1;
int duration        = 0;
int distance        = 0;

// Temp sensor
const int tempPin   = A0;
float temp_res      = 0;
float reading       = 0;
float voltage       = 0;

// Sound sensor
const int soundPin  = A2;

int soundPkPk       = 0;    // Pk-pk value of the incoming sound wave.
float soundVout     = 0;    // Sound ADC reading converted to voltage.
float soundDba      = 0;    // Calculated sound value in dBa.

// ----------------------------------------------------------------------------
// BLUETOOTH
// ----------------------------------------------------------------------------

// UUIDs for service + characteristics
const char* serviceUuid = "b4250500-fb4b-4746-b2b0-93f0e61122c6";
const char* temperature = "b4250501-fb4b-4746-b2b0-93f0e61122c6";
const char* sound       = "b4250502-fb4b-4746-b2b0-93f0e61122c6";
const char* movement    = "b4250503-fb4b-4746-b2b0-93f0e61122c6";
const char* security    = "b4250504-fb4b-4746-b2b0-93f0e61122c6";

// Set up characteristics
BleUuid sensorNodeService(serviceUuid);

BleCharacteristic temperatureCharacteristic("temperature", BleCharacteristicProperty::NOTIFY, temperature, serviceUuid, onDataReceived, (void*)temperature);
BleCharacteristic soundCharacteristic("sound", BleCharacteristicProperty::NOTIFY, sound, serviceUuid, onDataReceived, (void*)sound);
BleCharacteristic movementCharacteristic("movement", BleCharacteristicProperty::NOTIFY, movement, serviceUuid, onDataReceived, (void*)movement);
BleCharacteristic securityCharacteristic("security", BleCharacteristicProperty::WRITE, security, serviceUuid, onDataReceived, (void*)security);

// Static function for handling Bluetooth Low Energy callbacks
static void onDataReceived(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context) {
    // If the control node has changed state, update the functionality of the sensor node.
    if(context == security) 
    {
        b_is_security_mode = data[0];
    }
}

// ----------------------------------------------------------------------------
// SLEEP MODE
// ----------------------------------------------------------------------------
SystemSleepConfiguration sleep_config;

// ----------------------------------------------------------------------------
// SETUP
// ----------------------------------------------------------------------------
void setup() {
    // Add the Bluetooth characteristics.
    BLE.addCharacteristic(temperatureCharacteristic);
    BLE.addCharacteristic(soundCharacteristic);
    BLE.addCharacteristic(movementCharacteristic);
    BLE.addCharacteristic(securityCharacteristic);

    // Begin advertising
    BleAdvertisingData advData;
    advData.appendServiceUUID(sensorNodeService);
    BLE.advertise(&advData);

    // Initialise the sensor pins
    pinMode(fanPin, OUTPUT);

    pinMode(tempPin, INPUT); 

    pinMode(soundPin, INPUT); 

    pinMode(echoPin, INPUT); 
    pinMode(trigPin, OUTPUT);

    // Initialise sleep mode characteristics. Only able to wake from a change in temperature value (24degC threshold) or a BLE message, or if 60 minutes has passed.
    sleep_config.mode(SystemSleepMode::ULTRA_LOW_POWER).duration(60min).ble().analog(tempPin, 740, AnalogInterruptMode::CROSS);

    // Sets the RGB LED control so we can see what state the sensor node is in.
    RGB.control(true);

    curr_temp_val = analogRead(tempPin);
}

// ----------------------------------------------------------------------------
// LOOP
// ----------------------------------------------------------------------------
void loop() {

    // Calculate how many hours board has been on
    uint16_t hours_up = floor(millis()/MS_IN_HR) + 1; // Ceil not working properly?

    // Check if the power budget for the hour has been exceeded.
    b_in_power_budget = (total_pwr_consumption > hours_up*pwr_budget) ? 0 : 1;

    // Running in NORMAL mode
    if(!b_is_security_mode && b_in_power_budget)
    {
        // Set the LED to green.
        RGB.color(0x00,0xFF,0x00);

        // If it's been more than 10 minutes since a status update has been sent to the Cluster Head, send one.
        if((millis() - time_since_last_msg) > MS_IN_10_MIN)
        {
            // Send the updated information to the cluster head.
            // D[0] - D[1]:     Temp value         [degC]
            // D[2] - D[3]:     Power consumption   [mW]
            uint32_t tempData = 0;

            tempData |= curr_temp_val & 0xFF;
            tempData |= fan_pwr_consumption[curr_temp_lvl] << 16;

            temperatureCharacteristic.setValue(tempData);
            time_since_last_msg = millis();
        }

        // Determine the intensity that the fan needs to be.
        curr_temp_val = readTempLevel();

        if(curr_temp_val > 24)
        {
            curr_temp_lvl = F_HIGH;
        }

        else if((curr_temp_val >= 20) && (curr_temp_val <= 24))
        {
            curr_temp_lvl = F_LOW;
        }

        else if(curr_temp_val < 20)
        {
            curr_temp_lvl = F_OFF;
        }

        // If there has been a change in temperature state, update the CH.
        if (curr_temp_lvl != prev_temp_lvl)
        {
            // Calculate the total amount of time that the module has been running at the (now previous) fan intensity level.
            fan_pwr_time[1] = fan_pwr_time[0];
            fan_pwr_time[0] = millis(); // Used to calculate the amount of power consumed.
            float time_spent_sec = (fan_pwr_time[0] - fan_pwr_time[1]) * 1/1000; // ms * sec/ms

            // Increment the total power consumption.
            total_pwr_consumption += fan_pwr_consumption[prev_temp_lvl] * time_spent_sec;

            // Change the fan level.
            switch(curr_temp_lvl)
            {
                case F_LOW:
                {
                    analogWrite(fanPin, FAN_1);
                    break;
                }

                case F_HIGH:
                {
                    analogWrite(fanPin, FAN_2);
                    break;
                }

                case F_OFF:
                {
                    analogWrite(fanPin, FAN_OFF);
                    break;
                }

                default:
                {
                    break;
                }
            }

            // Send the updated information to the cluster head.
            // D[0] - D[1]:     Temp value         [degC]
            // D[2] - D[3]:     Power consumption   [mW]
            uint32_t tempData = 0;

            tempData |= curr_temp_val & 0xFF;
            tempData |= fan_pwr_consumption[curr_temp_lvl] << 16;
            temperatureCharacteristic.setValue(tempData);

            // Current state is now the previous state since the CH has been updated.
            prev_temp_lvl = curr_temp_lvl;
        }
    }

    // Running in throttled NORMAL mode
    else if(!b_is_security_mode && !b_in_power_budget)
    {
        // Set the LED to red.
        RGB.color(0xFF,0x00,0x00);

        // In throttled mode, temperature level thresholds and the resulting fan intensity have been altered to preserve battery life.
        // Additionally, if the fan is off, the node will go into sleep mode. It will only be able to be woken if the temperature ADC value cross the 30degC threshold or if the Cluster Head sends data.

        // Determine the intensity that the fan needs to be.
        curr_temp_val = readTempLevel();

        if(curr_temp_val > 30)
        {
            curr_temp_lvl = F_LOW;
        }

        else
        {
            curr_temp_lvl = F_OFF;
        }

        // If there has been a change in fan state, update the CH.
        if (curr_temp_lvl != prev_temp_lvl)
        {
            // Calculate the total amount of time that the module has been running at the (now previous) fan intensity level.
            fan_pwr_time[1] = fan_pwr_time[0];
            fan_pwr_time[0] = millis(); // Used to calculate the amount of power consumed.
            float time_spent_hr = (fan_pwr_time[0] - fan_pwr_time[1]) * 1/1000 * 1/60 * 1/60; // ms * sec/ms * min/sec * hour/min

            // Increment the total power consumption.
            total_pwr_consumption += fan_pwr_consumption[prev_temp_lvl] * time_spent_hr;

            // Change the fan level.
            switch(curr_temp_lvl)
            {
                case F_LOW:
                {
                    analogWrite(fanPin, FAN_1);
                    break;
                }

                case F_OFF:
                {
                    analogWrite(fanPin, FAN_OFF);
                    break;
                }

                default:
                {
                    break;
                }
            }

            // Send the updated information to the cluster head.
            // D[0] - D[1]:     Temp value         [degC]
            // D[2] - D[3]:     Power consumption   [mW]
            uint32_t tempData = 0;

            tempData |= curr_temp_val & 0xFF;
            tempData |= fan_pwr_consumption[curr_temp_lvl] << 16;
            temperatureCharacteristic.setValue(tempData);

            // Current state is now the previous state since the CH has been updated.
            prev_temp_lvl = curr_temp_lvl;
        }

        // If the temp is off, go into sleep mode.
        if(prev_temp_lvl == FAN_OFF)
        {
            System.sleep(sleep_config);
        }
    }

    // Running in SECURITY mode
    else
    {
        // Turn off the actuators.
        curr_temp_lvl = F_OFF;
        analogWrite(fanPin, FAN_OFF);

        // Set the LED to blue.
        RGB.color(0x00,0x00,0xFF);

         // Read the movement sensor

        // To prevent a false positive security event, make both values equal to the same value on the first read.
        if(b_is_first_dist_read)
        {
            b_is_first_dist_read = 0;
            curr_dist_val = readDistance();
            prev_dist_val = curr_dist_val;
        }

        prev_dist_val = curr_dist_val;
        curr_dist_val = readDistance();
        Serial.printf("current distance value is %d \n", curr_dist_val);

        // If the distance change was greater than 10cm, can assume that movement has occured.
        if(abs(curr_dist_val - prev_dist_val) > 10)
        {
            b_is_security_event = 1;
            eventTrigger = MOVEMENT;
        }

        // Read the sound sensor.
        curr_sound_val = readSoundLevel();

        if(curr_sound_val > 80)
        {
            b_is_security_event = 1;
            eventTrigger = SOUND;
        } 
        else if(curr_sound_val > 70)
        {
            // If this is the first time that it's been this sound level, begin counting the duration.
            if(!b_is_first_sound_instance)
            {
                b_is_first_sound_instance = 1;
                sound_start_time = millis();
            }
            
            else
            {
                unsigned long current_sound_time = millis();

                // If it's been longer than 10 seconds, report a security event.
                if((current_sound_time - sound_start_time) > 10000)
                {
                    sound_duration_sec = (uint8_t)((current_sound_time - sound_start_time)/1000);
                    b_is_security_event = 1;
                    b_is_first_sound_instance = 0;
                }
            }

        }
        else if((curr_sound_val <= 70) && (curr_sound_val >= 55))
        {
            // If this is the first time that it's been this sound level, begin counting the duration.
            if(!b_is_first_sound_instance)
            {
                b_is_first_sound_instance = 1;
                sound_start_time = millis();
            }
            
            else
            {
                unsigned long current_sound_time = millis();

                // If it's been longer than 30 seconds, report a security event.
                if((current_sound_time - sound_start_time) > 30000)
                {
                    sound_duration_sec = (uint8_t)((current_sound_time - sound_start_time)/1000);
                    b_is_security_event = 1;
                    b_is_first_sound_instance = 0;
                }
            }
        }

        // If a security event has been detected, notify the CH.
        if(b_is_security_event)
        {
            // Send the updated information to the cluster head.
            switch(eventTrigger)
            {
                // -- Sound event --
                // D[0]: Sound level [dbA]
                // D[1]: Sound duration [sec]
                case SOUND:
                {
                    uint16_t soundData = 0;
                    soundData |= (uint8_t)curr_sound_val;
                    soundData |= sound_duration_sec << 8;
                    soundCharacteristic.setValue(soundData);
                    break;
                }

                // -- Movement event --
                // D[0] - D[1]: Distance [cm]
                case MOVEMENT:
                {
                    Serial.printf("-------------------- JUST SENT MOVEMENT ");
                    uint16_t movementData = curr_dist_val;
                    movementCharacteristic.setValue(movementData);
                    break;
                }
                
                default: // Also covers NONE.
                {
                    break;
                }
            }

            // Reset the trigger now that the CH has been notified.
            eventTrigger = NONE;
            b_is_security_event = 0;
        }
    }
}

/*
 * @brief Reads the current temp level from the sensor.
 * @returns The resulting temp level in degC.
*/
long readTempLevel()
{
    unsigned int avgTempRead = 0;  // Summation of temperature reads.

    for (int i = 0; i < NUM_TEMP_READS; i++)
    {
        avgTempRead += analogRead(tempPin);
    }

    reading = avgTempRead/NUM_TEMP_READS;   // Read the ADC value for the light resistor.
    temp_res = (analogRead(tempPin) * 0.806) / 10;

    return temp_res;
}

/*
 * @brief Reads the current sound level from the sensor.
 * @returns The resulting sound level in dBa.
*/
float readSoundLevel()
{
    unsigned int tempSoundRead = 0;  // Summation of sound reads.
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
    soundVout = 3.3 * soundPkPk/4096;
    soundDba = 17.831 * log(soundVout) + 87.579;  

    return 10; // FIX ME
}

/*
 * @brief Reads the distance of an object from the sensor.
 * @returns The resulting distance in cm.
*/
int readDistance()
{
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

    return distance;
}
#endif

#ifdef CENTRAL

enum fanSpeed
{
    FANOFF,
    SPEED1,
    SPEED2
};

enum securityReporter
{
    NONE,
    SN1,
    SN2
};

enum securityState
{
    NORMAL,
    SECURITY
};

enum securityEvent
{
  NOEVENT,
  SOUND,
  MOVEMENT
};

// ----------------------------------------------------------------------------
// SECURITY
// ----------------------------------------------------------------------------
const int trigPin   = D2;

securityEvent security_event_type = NOEVENT;
securityReporter reporting_node = NONE;

uint16_t security_distance_cm   = 0;

uint8_t security_sound_dba      = 0;
uint8_t security_sound_duration_sec = 0;

// ----------------------------------------------------------------------------
// SYSTEM VARIABLES
// ----------------------------------------------------------------------------
securityState current_state = NORMAL;
securityState requested_state = NORMAL;

String fanSpeedStr[3] = {"OFF", "Speed 1", "Speed 2"};

fanSpeed sn2Fan = FANOFF;

uint16_t sn1_light_level_lux    = 0;
uint16_t sn1_power_usage_mW     = 0;

uint16_t sn2_temp_level_degc    = 0;
uint16_t sn2_power_usage_mW     = 0;

uint8_t intruder = 0;
unsigned long previousMillis_60 = 0;
unsigned long previousMillis_2 = 0;

int LED_Green = 6; // Green LED
int LED_Red = 7;   // Red LED
int ledState = LOW;


// ----------------------------------------------------------------------------
// BLUETOOTH
// ----------------------------------------------------------------------------
const size_t SCAN_RESULT_MAX = 30;
BleScanResult scanResults[SCAN_RESULT_MAX];
const size_t SCAN_RESULT_COUNT = 20;
const unsigned long SCAN_PERIOD_MS = 2000;
unsigned long lastScan = 0;


// SENSOR NODE 1
BleCharacteristic sn1LightCharacteristic;
BleCharacteristic sn1SoundCharacteristic;
BleCharacteristic sn1MovementCharacteristic;
BleCharacteristic sn1SecurityCharacteristic;
BlePeerDevice sn1Peer;

const BleUuid sn1ServiceUuid("b4250400-fb4b-4746-b2b0-93f0e61122c6");
const BleUuid sn1LightUuid("b4250401-fb4b-4746-b2b0-93f0e61122c6");
const BleUuid sn1SoundUuid("b4250402-fb4b-4746-b2b0-93f0e61122c6");
const BleUuid sn1MovementUuid("b4250403-fb4b-4746-b2b0-93f0e61122c6");
const BleUuid sn1SecurityUuid("b4250404-fb4b-4746-b2b0-93f0e61122c6");

// SENSOR NODE 2
BleCharacteristic sn2TemperatureCharacteristic;
BleCharacteristic sn2SoundCharacteristic;
BleCharacteristic sn2MovementCharacteristic;
BleCharacteristic sn2SecurityCharacteristic;
BlePeerDevice sn2Peer;

const BleUuid sn2ServiceUuid("b4250500-fb4b-4746-b2b0-93f0e61122c6");
const BleUuid sn2TemperatureUuid("b4250501-fb4b-4746-b2b0-93f0e61122c6");
const BleUuid sn2SoundUuid("b4250502-fb4b-4746-b2b0-93f0e61122c6");
const BleUuid sn2MovementUuid("b4250503-fb4b-4746-b2b0-93f0e61122c6");
const BleUuid sn2SecurityUuid("b4250504-fb4b-4746-b2b0-93f0e61122c6");

// BLUETOOTH
uint16_t security_mode = 0xFF;
uint16_t normal_mode = 0x00;
uint16_t timer = 10000;

// BLUETOOTH DATA CALLBACKS

void sn1LightCB(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context) {
    sn1_light_level_lux = (data[1] << 8) | data[0];
    sn1_power_usage_mW = (data[3] << 8) | data[2];
}

void sn1SoundCB(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context) {
    security_sound_dba = (data[1] << 8) | data[0];
    security_sound_duration_sec = (data[3] << 8) | data[2];

    reporting_node = SN1;
    security_event_type = SOUND;
    previousMillis_60 = millis();
}

void sn1MoveCB(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context) {
    security_distance_cm = (data[1] << 8) | data[0];

    reporting_node = SN1;
    security_event_type = MOVEMENT;
    previousMillis_60 = millis();
}

void sn2TempCB(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context) {
    sn2_temp_level_degc = (data[1] << 8) | data[0];
    sn2_power_usage_mW = (data[3] << 8) | data[2];
}

void sn2SoundCB(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context) {
    security_sound_dba = (data[1] << 8) | data[0];
    security_sound_duration_sec = (data[3] << 8) | data[2];

    previousMillis_60 = millis();

    reporting_node = SN2;
    security_event_type = SOUND;
}

void sn2MoveCB(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context) {
    security_distance_cm = (data[1] << 8) | data[0];

   previousMillis_60 = millis();

    reporting_node = SN2;
    security_event_type = MOVEMENT;
}

// ----------------------------------------------------------------------------
// LCD
// ----------------------------------------------------------------------------
DFRobot_LCD lcd(16, 2); // 16 characters and 2 rows

// ----------------------------------------------------------------------------
// SETUP
// ----------------------------------------------------------------------------
void setup() {
  Serial.begin();
	BLE.on();

    RGB.control(true);

    // Initialise all the characteristics available for the sensor nodes.
    sn1LightCharacteristic.onDataReceived(sn1LightCB, &sn1LightCharacteristic);
    sn1SoundCharacteristic.onDataReceived(sn1SoundCB, &sn1SoundCharacteristic);
    sn1MovementCharacteristic.onDataReceived(sn1MoveCB, &sn1MovementCharacteristic);

    sn2TemperatureCharacteristic.onDataReceived(sn2TempCB, &sn2TemperatureCharacteristic);
    sn2SoundCharacteristic.onDataReceived(sn2SoundCB, &sn2SoundCharacteristic);
    sn2MovementCharacteristic.onDataReceived(sn2MoveCB, &sn2MovementCharacteristic);

    // Initialise the sensor pins
    pinMode(trigPin, INPUT);
    pinMode(LED_Red, OUTPUT);
    pinMode(LED_Green, OUTPUT);
    digitalWrite(LED_Green, HIGH);
    digitalWrite(LED_Red, HIGH);

    // Initialise the LCD
    lcd.init();
    lcd.setBacklight(0);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("SMART HOME BY");
    lcd.setCursor(0, 1);
    lcd.print("Mitch & Mikyla");
}

void loop() {
    // Only run CH operations if both Sensor Nodes are connected to the Cluster Head.
    if (sn1Peer.connected() && sn2Peer.connected()) {
        RGB.color(0x00,0xFF,0x00);

        // See what the desired security state is for the Sensor Nodes.
        requested_state = (digitalRead(trigPin) == HIGH) ? SECURITY : NORMAL;

        // If there has been a change in operation state, notify the Sensor Nodes.
        if(requested_state != current_state)
        {
            lcd.clear();

            if(requested_state == SECURITY)
            {
                current_state = requested_state;

                sn1SecurityCharacteristic.setValue(security_mode);
                sn2SecurityCharacteristic.setValue(security_mode);
            }

            else if (requested_state == NORMAL)
            {
                current_state = requested_state;
                
                sn1SecurityCharacteristic.setValue(normal_mode);
                sn2SecurityCharacteristic.setValue(normal_mode);
            }
        }
        
        if(current_state == NORMAL){


            if(sn2_temp_level_degc < 20)
                sn2Fan = FANOFF;
            if(sn2_temp_level_degc > 24)
                sn2Fan = SPEED2;
            if((sn2_temp_level_degc >= 20) && (sn2_temp_level_degc <= 24))
                sn2Fan = SPEED1;
   
            lcd.setCursor(0, 0);
            lcd.print("TEMP: ");
            lcd.print(sn2_temp_level_degc); // Temperature Value
            lcd.print("DEG C ");//OFFSET spaces for scrolling
            lcd.print("FAN ");
            lcd.print(fanSpeedStr[sn2Fan]);//FAN ENUM: OFF, SPEED 1, SPEED 2
            lcd.print(" ");
            lcd.print(sn2_power_usage_mW); //Fan power Consumption
            lcd.print("mW"); //OFFSET spaces for scrolling
            lcd.print("  "); //OFFSET spaces for scrolling

            lcd.setCursor(0, 1);
            lcd.print("LIGHT LEVEL: ");
            lcd.print(sn1_light_level_lux);// LUX level
            lcd.print(" LUX ");
            lcd.print(sn1_power_usage_mW); //LED Power Consumption 
            lcd.print("mW    "); //OFFSET spaces for scrolling
            lcd.scrollDisplayLeft();

        }

       else if(current_state == SECURITY){

            if(security_event_type == NOEVENT){
            
                lcd.setCursor(0, 0);
                lcd.print("NO EVENT DETECTED      ");
                lcd.setCursor(0, 1);
                 lcd.print("                ");
                lcd.scrollDisplayLeft();

            }

            else if(security_event_type == MOVEMENT){
                //Serial.print("SEC MOV");
                
                lcd.setCursor(0, 0);
                lcd.print("SN: ");
                lcd.print(reporting_node);
                lcd.print(" Detected Movement ");
                lcd.setCursor(0, 1);
                lcd.print("Distance: ");
                lcd.print(security_distance_cm);
                lcd.print(" CM   ");
                
                Led_Flash_Green(2000);
                lcd.scrollDisplayLeft();
            }
           
             else if(security_event_type == SOUND){
                if((security_sound_dba > 55) && (security_sound_dba < 70) && (security_sound_duration_sec > 30)){
                lcd.setCursor(0, 0);
                lcd.print("SENSOR NODE: ");
                lcd.print(reporting_node);
                lcd.print(" Detected SOUND ");
                lcd.setCursor(0, 1);
                lcd.print("LEVEL: ");
                lcd.print(security_sound_dba);
                lcd.print(" DUR: ");
                lcd.print(security_sound_duration_sec);
                Led_Flash_Green(1000);
                lcd.scrollDisplayLeft();
                }
            
              else if((security_sound_dba > 70)  && (security_sound_duration_sec > 10)){
                lcd.setCursor(0, 0);
                lcd.print("SENSOR NODE: ");
                lcd.print(reporting_node);
                lcd.print(" Detected SOUND ");
                lcd.setCursor(0, 1);
                lcd.print("LEVEL: ");
                lcd.print(security_sound_dba);
                lcd.print("DUR: ");
                lcd.print(security_sound_duration_sec);
                Led_Flash_Red(1000);
                lcd.scrollDisplayLeft();
              }

              else if((security_sound_dba > 80) ){
                lcd.setCursor(0, 0);
                lcd.print("SENSOR NODE: ");
                lcd.print(reporting_node);
                lcd.print(" Detected SOUND ");
                lcd.setCursor(0, 1);
                lcd.print("Sound LEVEL: ");
                lcd.print(security_sound_dba);
                Led_Flash_Red(500);
                lcd.scrollDisplayLeft();
              }
             }
            }
    }
    
    else {
        RGB.color(0x00,0x00,0xFF);
        // Need to establish connections with both the sensor nodes.
    	if (millis() - lastScan >= SCAN_PERIOD_MS) {
    		// Time to scan
    		lastScan = millis();

    		size_t count = BLE.scan(scanResults, SCAN_RESULT_COUNT);
			if (count > 0) {
				for (uint8_t ii = 0; ii < count; ii++) {
					BleUuid foundServiceUuid;
					size_t svcCount = scanResults[ii].advertisingData().serviceUUID(&foundServiceUuid, 1);
					if (svcCount > 0 && foundServiceUuid == sn1ServiceUuid) {
						sn1Peer = BLE.connect(scanResults[ii].address());
						if (sn1Peer.connected()) {
							sn1Peer.getCharacteristicByUUID(sn1LightCharacteristic, sn1LightUuid);
							sn1Peer.getCharacteristicByUUID(sn1SoundCharacteristic, sn1SoundUuid);
                            sn1Peer.getCharacteristicByUUID(sn1MovementCharacteristic, sn1MovementUuid);
                            sn1Peer.getCharacteristicByUUID(sn1SecurityCharacteristic, sn1SecurityUuid);
						}
					}
                    if (svcCount > 0 && foundServiceUuid == sn2ServiceUuid){
                        sn2Peer = BLE.connect(scanResults[ii].address());
						if (sn2Peer.connected()) {
							sn2Peer.getCharacteristicByUUID(sn2TemperatureCharacteristic, sn2TemperatureUuid);
							sn2Peer.getCharacteristicByUUID(sn2SoundCharacteristic, sn2SoundUuid);
                            sn2Peer.getCharacteristicByUUID(sn2MovementCharacteristic, sn2MovementUuid);
                            sn2Peer.getCharacteristicByUUID(sn2SecurityCharacteristic, sn2SecurityUuid);
						}
				    }
			    }
    	    }

        }
    }
}

void Led_Flash_Green(unsigned long frequency){

unsigned long currentMillis = millis();
unsigned long currentMillis2 = millis();



    if (currentMillis - previousMillis_60 >= 60000)
    {
      reporting_node = NONE;
      security_event_type = NOEVENT;
     digitalWrite(LED_Green, HIGH);
    }

   if (currentMillis2 - previousMillis_2 >= frequency)
  {
    digitalWrite(LED_Red, HIGH);
    previousMillis_2 = currentMillis2;

    if (ledState == LOW)
    {
      ledState = HIGH;
    }
    else
    {
      ledState = LOW;
    }
    // digitalWrite(LED_Red, ledState);
    digitalWrite(LED_Green, !ledState);
  }
}

void Led_Flash_Red(unsigned long frequency){

unsigned long currentMillis = millis();
unsigned long currentMillis2 = millis();



    if (currentMillis - previousMillis_60 >= 10000)
    {
      reporting_node = NONE;
      security_event_type = NOEVENT;
      digitalWrite(LED_Green, HIGH);
     
    }

  if (currentMillis2 - previousMillis_2 >= frequency)
  {
    digitalWrite(LED_Green, HIGH);

    previousMillis_2 = currentMillis2;

    if (ledState == LOW)
    {
      ledState = HIGH;
    }
    else
    {
      ledState = LOW;
    }
     
    digitalWrite(LED_Green, !ledState);
  }
}
#endif