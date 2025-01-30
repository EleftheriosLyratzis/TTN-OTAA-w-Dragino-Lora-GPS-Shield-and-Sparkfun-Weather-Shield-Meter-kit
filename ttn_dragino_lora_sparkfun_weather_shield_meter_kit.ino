/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!

 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>                                        //I2C needed for sensors
#include "SparkFunMPL3115A2.h"                           //Pressure sensor - Search "SparkFun MPL3115" and install from Library Manager
#include "SparkFun_Si7021_Breakout_Library.h"            //Humidity sensor - Search "SparkFun Si7021" and install from Library Manager
#include "SparkFun_Weather_Meter_Kit_Arduino_Library.h"  //Weather meter kit - Search "SparkFun Weather Meter" and install from Library Manager
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
static const u1_t PROGMEM APPEUI[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
static const u1_t PROGMEM DEVEUI[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; //LSB Format
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
static const u1_t PROGMEM APPKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; //MSB Format
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Hardware pin definitions
// digital I/O pins
const byte WSPEED = 23;
const byte RAIN = 22;
// analog I/O pins
const byte REFERENCE_3V3 = A3;
const byte LIGHT = A1;
const byte BATT = A2;
const byte WDIR = A0;
//-=-=-=-=-=-=-=-=-=-=-=
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Global Variables
float humidity = 0;  // [%]
float tempC = 0;     // [temperature C]
float pressure = 0;
float wind_dir = 0;    // [degrees (Cardinal)]
float wind_speed = 0;  // [kph]
float rain = 0;        // [mm]
float batt_lvl = 11.8;  //[analog value from 0 to 1023]
float light_lvl = 455;  //[analog value from 0 to 1023]
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
static uint8_t mydata[10]; // Payload buffer
static osjob_t sendjob;
// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
};
MPL3115A2 myPressure;                                      //Create an instance of the pressure sensor
SI7021 myHumidity;                                        //Create an instance of the humidity sensor
SFEWeatherMeterKit myweatherMeterKit(WDIR, WSPEED, RAIN);  // Create an instance of the weather meter kit
void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));

            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void sendData() {
    humidity = myHumidity.getRH(); // Calc humidity from Si7021 sensor
    tempC = myHumidity.getTemperature(); //Calc temp from Si7021 sensor
    pressure = myPressure.readPressure()*0.01; //Calc pressure from MPL3115A2
    rain = myweatherMeterKit.getTotalRainfall(); // Calc Rain
    wind_speed = myweatherMeterKit.getWindSpeed(); // Calc Wind Speed
    wind_speed = kmphToBft(wind_speed);
    int current_water_level = getwaterLevel(); //Calc water Level
    int hum = (int)(humidity*100);
    Serial.println(hum);
    int temp = (int)(tempC*100);
    Serial.print(String(" ") + tempC);
    int pres = (int)(pressure);
    Serial.print(String(" ") + pressure);
    int prcp = (int)rain;
    Serial.print(String(" ") + rain);
    int wspd = (int)wind_speed;
    Serial.print(String(" ") + wind_speed);

    mydata[0] = hum >> 8;        // High byte of var1
    mydata[1] = hum & 0xFF;      // Low byte of var1
    mydata[2] = temp >> 8;        // High byte of var2
    mydata[3] = temp & 0xFF;      // Low byte of var2
    mydata[4] = pres >> 8;      // High byte of var3
    mydata[5] = pres & 0xFF;     // High byte of var3
    mydata[6] = prcp >> 8;      // High byte of var4
    mydata[7] = prcp & 0xFF;     // High byte of var4
    mydata[8] = wspd >> 8;      // High byte of var5
    mydata[9] = wspd & 0xFF;     // High byte of var5
    // Prepare the payload for transmission
    LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
    Serial.println(F("Packet queued"));
}

void do_send(osjob_t* j){
 sendData();
    // Schedule next transmission
    os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
}

void setup() {
    Serial.begin(9600);
    Wire.begin();        // Join i2c bus
    myPressure.begin();               // Get sensor online
    myPressure.setModeBarometer();    // Measure pressure in Pascals from 20 to 110 kPa
    myPressure.setOversampleRate(7);  // Set Oversample to the recommended 128
    myPressure.enableEventFlags();    // Enable all three pressure and temp event flags
    myHumidity.begin();               //Configure the humidity sensor
    myweatherMeterKit.setADCResolutionBits(10); // Configuring a 10-bit ADC resolution for the ATmega328 (RedBoard/Uno)
    myweatherMeterKit.begin(); // Begin weather meter kit
    Serial.println("Weather Shield online!");
    Serial.println(F("Starting"));

    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}

float get_light_level() {
  float operatingVoltage = analogRead(REFERENCE_3V3);
  float lightSensor = analogRead(LIGHT);
  operatingVoltage = 3.3 / operatingVoltage;  //The reference voltage is 3.3V
  lightSensor = operatingVoltage * lightSensor;
  return (lightSensor);
}

float get_battery_level() {
  float operatingVoltage = analogRead(REFERENCE_3V3);
  float rawVoltage = analogRead(BATT);
  operatingVoltage = 3.30 / operatingVoltage;  //The reference voltage is 3.3V
  rawVoltage = operatingVoltage * rawVoltage;  //Convert the 0 to 1023 int to actual voltage on BATT pin
  rawVoltage *= 4.90;  //(3.9k+1k)/1k - multiple BATT voltage by the voltage divider to get actual system voltage
  return (rawVoltage);
}

float kmphToBft(float kmph) { //Convert Wind Speed in Kmph to Bfts
  float thresholds[] = {0, 1, 6, 12, 20, 29, 39, 50, 62, 75, 89, 103, 118};
  int n = sizeof(thresholds) / sizeof(thresholds[0]); 
  for (int i = 1; i < n; i++) { 
    if (kmph < thresholds[i]) { 
      float rangeStart = thresholds[i - 1]; 
      float rangeEnd = thresholds[i]; 
      float rangeMidpoint = (rangeStart + rangeEnd) / 2; 
      float increment = (kmph - rangeStart) / (rangeMidpoint - rangeStart) * 0.5;
      return (i - 1) + increment; 
  }
  return 12; 
}}