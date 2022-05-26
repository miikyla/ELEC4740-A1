#define SN1
//#define SN2
//#define CENTRAL

#include "Particle.h"
#include "dct.h"
#include "math.h"

#define NUM_LIGHT_READS 100
#define NUM_SOUND_READS 500

#define LIGHT_50 128
#define LIGHT_75 191
#define LIGHT_100 255

enum lightLevel
{
    OFF,
    LOW,
    MED,
    HIGH
};

SYSTEM_MODE(MANUAL);

#ifdef SN1
// ----------------------------------------------------------------------------
// SYSTEM VARIABLES
// ----------------------------------------------------------------------------
// System state
bool b_is_security_mode     = 0;

long curr_light_val         = 0;
lightLevel prev_light_lvl   = OFF;
lightLevel curr_light_lvl   = OFF;

float curr_sound_val    = 0;
int curr_dist_val       = 0;

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
    pinMode(lightPin, INPUT); 

    pinMode(soundPin, INPUT); 

    pinMode(echoPin, INPUT); 
    pinMode(trigPin, OUTPUT);
}

// ----------------------------------------------------------------------------
// LOOP
// ----------------------------------------------------------------------------
void loop() {
    // Running in NORMAL mode
    if(!b_is_security_mode)
    {
        // If the light level has changed into a different state, change the lamp intensity and report back to the CH.
        curr_light_val = readLightLevel();

        if(curr_light_val > 400)
        {
            analogWrite(lampPin, LIGHT_50);
        } 
        else if((curr_light_val > 200) && (curr_light_val < 400))
        {
            analogWrite(lampPin, LIGHT_75);
        }
        else if(curr_light_val < 200)
        {
            analogWrite(lampPin, LIGHT_100);
        }
    }

    // Running in SECURITY mode
    else
    {
        // Read the movement sensor

        // If a change has been detected, notify the CH.

        // Read the sound sensor.

        // If a change has been detected, notify the CH.
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
