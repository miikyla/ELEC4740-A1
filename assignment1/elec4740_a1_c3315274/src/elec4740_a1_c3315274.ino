#define SN1
//#define SN2
//#define CENTRAL

#include "Particle.h"
#include "math.h"

#define MS_IN_HR 3600000

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

SYSTEM_MODE(MANUAL);

#ifdef SN1
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
float total_pwr_consumption         = 0;                // Lifetime power consumption of the device since bootup [mWh]
const float pwr_budget              = 2100;             // Maximum amount of power consumption allowed per hour before power management kicks and throttles NORMAL mode [mWh]

float curr_sound_val        = 0;
uint8_t sound_duration_sec  = 0;
bool b_is_first_sound_instance  = 0;
unsigned long sound_start_time  = 0;

int curr_dist_val           = 0;
int prev_dist_val           = 0; 
bool b_is_first_dist_read   = 1;

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

BleCharacteristic lightCharacteristic("light", BleCharacteristicProperty::NOTIFY, light, serviceUuid, onDataReceived, (void*)light);
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
}

// ----------------------------------------------------------------------------
// LOOP
// ----------------------------------------------------------------------------
void loop() {

    // Calculate how many hours board has been on
    uint16_t hours_up = floor(millis()/MS_IN_HR);

    // Check if the power budget for the hour has been exceeded.
    if(total_pwr_consumption > hours_up*pwr_budget)
    {
        b_in_power_budget = 0;
    }

    // Running in NORMAL mode
    if(!b_is_security_mode && b_in_power_budget)
    {
        // Set the LED to green.
        RGB.color(0x00,0xFF,0x00);

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
            float time_spent_hr = (light_pwr_time[0] - light_pwr_time[1]) * 1/1000 * 1/60 * 1/60; // ms * sec/ms * min/sec * hour/min

            // Increment the total power consumption.
            total_pwr_consumption += light_pwr_consumption[prev_light_lvl] * time_spent_hr;

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

            lightData |= curr_light_val & 0xFF;
            lightData |= light_pwr_consumption[curr_light_lvl] << 16;
            lightCharacteristic.setValue(lightData);

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
                    break;
                }

                // -- Movement event --
                // D[0] - D[1]: Distance [cm]
                case MOVEMENT:
                {
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

    return soundDba;
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
// UUIDs for service + characteristics
const char* serviceUuid = "b4250500-fb4b-4746-b2b0-93f0e61122c6"; //service
const char* red         = "b4250501-fb4b-4746-b2b0-93f0e61122c6"; //red char
const char* status      = "b4250504-fb4b-4746-b2b0-93f0e61122c6"; //status char

// Set the RGB BLE service
BleUuid rgbService(serviceUuid);

bool colour_state = 0;
uint16_t r1 = 3;
uint16_t r2 = 4;

// Set up characteristics
BleCharacteristic redCharacteristic("red", BleCharacteristicProperty::NOTIFY, red, serviceUuid, onDataReceived, (void*)red);
BleCharacteristic statusCharacteristic("status", BleCharacteristicProperty::WRITE, status, serviceUuid, onDataReceived, (void*)status);


// Static function for handling Bluetooth Low Energy callbacks
static void onDataReceived(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context) {
  // Sets the global level
  if( context == status ) 
  {
    RGB.color(0x00, data[0], 0x00);
  }
  Serial.printf("%d \n", data[0]);

}

// setup() runs once, when the device is first turned on.
void setup() {

  // Enable app control of LED
  RGB.control(true);

  // Add the characteristics
  BLE.addCharacteristic(redCharacteristic);
  BLE.addCharacteristic(statusCharacteristic);

  // Advertising data
  BleAdvertisingData advData;

  // Add the RGB LED service
  advData.appendServiceUUID(rgbService);

  // Start advertising!
  BLE.advertise(&advData);
}

// loop() runs over and over again, as quickly as it can execute.
void loop() {
    if (colour_state)
    {
        redCharacteristic.setValue(r1);
    }

    else
    {
        redCharacteristic.setValue(r2);
    }

    delay(5000);
    colour_state = !colour_state;
}
#endif

#ifdef CENTRAL
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
bool led_state = 0;
uint16_t led_on = 0xFF;
uint16_t led_off = 0x00;
uint16_t timer = 10000;

// BLUETOOTH DATA

void onDataReceived(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context) {
    Serial.printf("%d \n", data[0]);
}

void setup() {
    Serial.begin();
	BLE.on();

    RGB.control(true);

    sn1RedCharacteristic.onDataReceived(onDataReceived, &sn1RedCharacteristic);
    sn2RedCharacteristic.onDataReceived(onDataReceived, &sn2RedCharacteristic);
}

void loop() {
    if (BLE.connected()) {
        if (timer > 0)
        {
            timer--;
        }

        else
        {
            timer = 10000;

            if (led_state)
            {
                sn1StatusCharacteristic.setValue(led_on);
                sn2StatusCharacteristic.setValue(led_on);
            }

            else
            {
                sn1StatusCharacteristic.setValue(led_off);
                sn2StatusCharacteristic.setValue(led_off);
            }

            led_state = !led_state;
        }
    }
    
    else {
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
							sn1Peer.getCharacteristicByUUID(sn1RedCharacteristic, sn1RedUuid);
							sn1Peer.getCharacteristicByUUID(sn1StatusCharacteristic, sn1StatusUuid);
                            RGB.color(0xFF, 0x00, 0x00);
						}
					}
                    if (svcCount > 0 && foundServiceUuid == sn2ServiceUuid)
                        sn2Peer = BLE.connect(scanResults[ii].address());
						if (sn2Peer.connected()) {
                            sn2Peer.getCharacteristicByUUID(sn2RedCharacteristic, sn2RedUuid);
							sn2Peer.getCharacteristicByUUID(sn2StatusCharacteristic, sn2StatusUuid);
                            RGB.color(0x00, 0xFF, 0x00);
						}

                    else
                    {
                        RGB.color(0x00, 0x00, 0xFF);
                    }
				}
			}
    	}

    }
}
#endif
