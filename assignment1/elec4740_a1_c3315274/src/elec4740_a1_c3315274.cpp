/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#line 1 "c:/Users/Mikyla/Documents/GitHub/ELEC4740-A1/assignment1/elec4740_a1_c3315274/src/elec4740_a1_c3315274.ino"
//#define SN1
//#define SN2
#define CENTRAL

#include "Particle.h"
static void onDataReceived(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context);
void setup();
void loop();
static void onDataReceived(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context);
void setup();
void loop();
void onDataReceived(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context);
void setup();
void loop();
#line 6 "c:/Users/Mikyla/Documents/GitHub/ELEC4740-A1/assignment1/elec4740_a1_c3315274/src/elec4740_a1_c3315274.ino"

SYSTEM_MODE(MANUAL);

#ifdef SN1
// UUIDs for service + characteristics
const char* serviceUuid = "b4250400-fb4b-4746-b2b0-93f0e61122c6"; //service
const char* red         = "b4250401-fb4b-4746-b2b0-93f0e61122c6"; //red char
const char* status      = "b4250404-fb4b-4746-b2b0-93f0e61122c6"; //status char

// Set the RGB BLE service
BleUuid rgbService(serviceUuid);

bool colour_state = 0;
uint16_t r1 = 1;
uint16_t r2 = 2;

// Set up characteristics
BleCharacteristic redCharacteristic("red", BleCharacteristicProperty::NOTIFY, red, serviceUuid, onDataReceived, (void*)red);
BleCharacteristic statusCharacteristic("status", BleCharacteristicProperty::WRITE, status, serviceUuid, onDataReceived, (void*)status);


// Static function for handling Bluetooth Low Energy callbacks
static void onDataReceived(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context) {
  // Sets the global level
  if( context == status ) 
  {
    RGB.color(data[0], 0x00, 0x00);
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
BleCharacteristic sn1RedCharacteristic;
BleCharacteristic sn1StatusCharacteristic;
BlePeerDevice sn1Peer;

const BleUuid sn1ServiceUuid("b4250400-fb4b-4746-b2b0-93f0e61122c6");
const BleUuid sn1RedUuid("b4250401-fb4b-4746-b2b0-93f0e61122c6");
const BleUuid sn1StatusUuid("b4250404-fb4b-4746-b2b0-93f0e61122c6");

// SENSOR NODE 2
BleCharacteristic sn2RedCharacteristic;
BleCharacteristic sn2StatusCharacteristic;
BlePeerDevice sn2Peer;

const BleUuid sn2ServiceUuid("b4250500-fb4b-4746-b2b0-93f0e61122c6");
const BleUuid sn2RedUuid("b4250501-fb4b-4746-b2b0-93f0e61122c6");
const BleUuid sn2StatusUuid("b4250504-fb4b-4746-b2b0-93f0e61122c6");

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
