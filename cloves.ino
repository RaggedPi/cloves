/*
  RaggedPi Project
  Arduino "Cloves" - Weatherstation
  Written by david durost <david.durost@gmail.com>
*/
/* Includes */
#include <SoftwareSerial.h>                 // Serial library
#include <XBee.h>                           // XBee library
#include <Wire.h>                           // One Wire library
#include <SDL_Weather_80422.h>              // Weather Station library
#include <Adafruit_ADS1015.h>               // library
#include <Adafruit_HTU21DF.h>               // Humidity library
#include <SPI.h>                            // SPI library
#include <PWFusion_AS3935.h>                // Lightning Detection library
#include <SFE_BMP180.h>                     // Barometric Pressure library
#include <Time.h>                           // Time library

/* Output Settings */
//#define SERIAL                              // uncomment for serial output
#define XBEE                                // uncomment for xbee2 output
//#define DEBUG                               // uncomment for debug output

/* Constants */
// Misc
#define SDA 20                              // sda
#define SDL 21                              // sdl
#define CS 53                               // chipselect
#define LED 13                              // led pin
#define SI 4                                // select input
#define C 0                                 // delimiter
#define F 1                                 // delimiter
#define CURRENT 0                           // delimiter
#define PREVIOUS 1                          // delimiter
// Barometric pressure
#define ABS 0                               // delimiter
#define SEA 1                               // delimiter
#define ABS_HG 2                            // delimiter
#define SEA_HG 3                            // delimiter
#define MB_TO_INHG 0.0295333727             // conversion factor
    // Weatherstation
#define ANEM_PIN 18                         // digital pin
#define ANEM_INT 5                          // int                 
#define RAIN_PIN 2                          // digital pin
#define RAIN_INT 1                          // int
#define SAMPLE_TIME 5.0                     // float
#define MODE SDL_MODE_SAMPLE                // sample mode
#define ALTITUDE 145.0                      // current altitude in meters
#define ALTITUDE_F (_m2ft(ALTITUDE))        // current altitude in feet (m*3.28084)
// Lightning arrestor
#define AS3935_PIN 2                        // analog pin AS3935 IRQ
#define AS3935_IN 0                         // delimiter
#define AS3935_OUT 1                        // delimiter
#define AS3936_DIST_DIS 0                   // delimiter
#define AS3935_DIST_EN 1                    // delimiter
#define AS3935_CAPACITANCE 104              // on board
// Xbee2
#define XB_ADDR1 0x0013a200                 // hex
#define XB_ADDR2 0x40682fa2                 // hex
// Battery
#define BATTERY_PIN A0                      // analog pin
#define READ_TIME 60000                     // ms

/* Interruptors */
void AS3935_ISR();                          // lightning arrestor

/* Variables */
double cTemp = 0.0;                         // current temperature
double hTemp = 0.0;                         // high temperature
double lTemp = 0.0;                         // low temperature
double bmp[] = { 0.0, 0.0, 0.0, 0.0 };      // barometric pressure
float windGust[2] = { 0.0, 0.0 };            // weather station
float windGustDirection[2] = { 0.0, 0.0 };  // weather station
float windDirection[2] = { 0.0, 0.0 };      // weather station
float windSpeed[2] = { 0.0 };               // weather station
float rainfall = 0.0;                       // weather station
float batVoltage;                           // battery
volatile int AS3935_ISR_Trig = 0;           // lightning arrestor
uint8_t payload[] = {0};                    // xbee
unsigned long lastReadTime = 0;                 // misc

/* Objects */
SDL_Weather_80422 weatherStation(
    ANEM_PIN, RAIN_PIN, ANEM_INT, RAIN_INT, A0, SDL_MODE_INTERNAL_AD
);                                          // weather station
Adafruit_HTU21DF htu = Adafruit_HTU21DF();  // humidity sensor
PWF_AS3935 AS3935(CS, AS3935_PIN, SI);      // lightning arrestor
SFE_BMP180 barometricPressure;              // barometric pressure
XBee xbee = XBee();                         // xbee
XBeeAddress64 addr64 = XBeeAddress64(XB_ADDR1, XB_ADDR2); // xbee address
ZBTxRequest zbTx = ZBTxRequest(addr64, payload, sizeof(payload)); // xbee request
ZBTxStatusResponse txStatus = ZBTxStatusResponse(); // xbee response

/* Utility Methods */
/**
 * Meters to feet conversiuon
 * @param int meters
 * @return int
 */
int _m2ft(int meters)
{
    return (meters * 3.28084);
}

/**
 * Kilometers to feet conversion
 * @param  int km
 * @return int
 */
int _km2ft(int km) {
    return (_m2ft(km/1000));
}

/**
 * Kilometers to miles conversion
 * @param  int km 
 * @return int
 */
int _km2mi(int km) {
    return (km * 0.62137);
}

/**
 * Celsius to Fahrenheit
 * @param  float celsius
 * @return float
 */
float _c2f(float celsius) {
    return ((9.0/5.0)*celsius)+32.0;
}

/**
 * Build data payload
 * @param  uint8_t distance distance of a lightning strike 
 *                          else weather payload data
 
 */
void buildData(uint8_t distance=0) {
    if (0 == distance) {
        float h = htu.readHumidity();
        payload[0] = 0 & 0xff;
        payload[1] = (uint8_t)_c2f(cTemp) >> 8 & 0xff;
        payload[2] = (uint8_t)_c2f(cTemp) & 0xff;
        payload[3] = (uint8_t)h >> 8 & 0xff;
        payload[4] = (uint8_t)h & 0xff;
        payload[5] = (uint8_t)bmp[ABS_HG] >> 8 & 0xff;
        payload[6] = (uint8_t)bmp[ABS_HG] & 0xff;
        payload[7] = (uint8_t)windSpeed[CURRENT] >> 8 & 0xff;
        payload[8] = (uint8_t)windSpeed[CURRENT] & 0xff;
        payload[9] = (uint8_t)windDirection[CURRENT] >> 8 & 0xff;
        payload[10] = (uint8_t)windDirection[CURRENT] & 0xff;
        payload[11] = (uint8_t)windGust[CURRENT] >> 8 & 0xff;
        payload[12] = (uint8_t)windGust[CURRENT] & 0xff;
        payload[13] = (uint8_t)rainfall >> 8 & 0xff;
        payload[14] = (uint8_t)rainfall & 0xff; 
    } else {
        payload[0] = 1 & 0xff;
        payload[1] = distance & 0xff;
    }
}

/**
 * Send XBEE data
 */
void sendData() {
    xbee.send(zbTx);
    if (xbee.readPacket(500)) {
        if (xbee.getResponse().getApiId() == ZB_TX_STATUS_RESPONSE) {
            xbee.getResponse().getZBTxStatusResponse(txStatus);
            if (txStatus.getDeliveryStatus() == SUCCESS) {
                // yay!
            } else {
                // boo.
            }
        }
    } else if (xbee.getResponse().isError()) {
        // error...error...error...
    } else {
        // ...?
    }
}

/**
 * Get wind direction string
 * @param  float heading
 * @param  bool  longform
 * @return char*
 */
String getWindDirectionStr(float heading, bool longform = false) {
    String directions[] = {
        "n", "nne", "ne", "ene", 
        "e", "ese", "se", "sse",
        "s", "ssw", "sw", "wsw",
        "w", "wnw", "nw", "nnw",
        "n"
    }; 
    String directions_long[] = {
        "north", "north northeast",
        "northeast", "east northeast",
        "east", "east southeast",
        "southeast", "south southeast",
        "south", "south southwest",
        "southwest", "west southwest",
        "west", "west northwest",
        "northwest", "north northwest",
        "north"
    };
    int i = round(heading);
    i = (round(i % 360) / 22.5)+1;
    if (longform)   return directions_long[i];
    return directions[i];
}

/**
 * AS3935 interrupt handler
 */
void doAS3935() {
    // reset flag
    AS3935_ISR_Trig = 0; 

    switch (AS3935.AS3935_GetInterruptSrc()) {
        case 0:
            #ifdef SERIAL
            Serial.println("[ERROR] AS3935 interrupt source result not expected.");
            #endif
            break;
        case 1: {
            String msg;
            uint8_t distance = AS3935.AS3935_GetLightningDistKm();
            // 1 - near, 63 - distant
            if (1 == distance) {
                #ifdef SERIAL
                Serial.println("Lightning overhead!");
                #endif
            } 
            else if (63 >= distance) {
                #ifdef SERIAL
                Serial.println("Out of range lightning detected.");
                #endif
            }
            else if (63 > distance && 1 < distance) {
                #ifdef SERIAL
                Serial.print("Lightning detected! (");
                Serial.print(distance);
                Serial.println("km)");
                #endif
            }
            #ifdef XBEE
            buildData(distance);
            sendData();
            #endif  
            break;
        }
        case 2:
            Serial.println("[ERROR] AS3935 Disturber detected");
            break;
        case 3:
            Serial.println("[ERROR] AS3935 Noise level too high.");
            break;    
        default:
            break;
    }        
}

/**
 * AS3935 interrupt handler
 */
void AS3935_ISR() {
    AS3935_ISR_Trig = 1;
}

/* Read Methods */
/**
 * Read anemometer
 */
void readAnemometer() {
    windSpeed[CURRENT] = weatherStation.current_wind_speed()/1.6;
    windDirection[CURRENT] = weatherStation.current_wind_direction();
    
    windGust[CURRENT] = weatherStation.get_wind_gust()/1.6;
    windGustDirection[CURRENT] = windDirection[CURRENT];

    if (windSpeed[CURRENT] > windSpeed[PREVIOUS]) {
        windSpeed[PREVIOUS] = windSpeed[CURRENT];
        windDirection[PREVIOUS] = windDirection[CURRENT];
    }

    if (windGust[CURRENT] > windGust[PREVIOUS]) {
        windGust[PREVIOUS] = windGust[CURRENT];
        windGustDirection[PREVIOUS] = windGustDirection[CURRENT];
    }    
}
/**
 * Read temperature
 */
void readTemperature() {
    if (0 != barometricPressure.startTemperature()) {
        delay(5);
        if (0 == barometricPressure.getTemperature(cTemp))
            Serial.println("[WARN] Could not read temperature");
    } else Serial.println("[WARN] Could not start temperature sensor.");
}

/**
 * Read barometric pressure
 */
void readBarometricPressure() {
    int p;
    if(0 == cTemp)  p = barometricPressure.getTemperature(cTemp);
    p = barometricPressure.startPressure(3); // [0-3] resolution, return ms read time
    if (0 != p) {
        delay(p);
        if (0 != barometricPressure.getPressure(bmp[ABS], cTemp)) {
            bmp[SEA] = barometricPressure.sealevel(bmp[ABS], ALTITUDE);
            bmp[ABS_HG] = bmp[ABS] * MB_TO_INHG;
            bmp[SEA_HG] = bmp[SEA] * MB_TO_INHG; 
        } else Serial.println("[WARN] Could not get pressure.");
    } else Serial.println("[WARN] Could not start pressure sensor.");
}

/**
 * Read battery voltage
 */
void readBatteryVoltage() {
    int v = analogRead(BATTERY_PIN);

    batVoltage = (float(v)*5)/1023*2;
}

/* Core Methods */
/**
 * Setup
 */
void setup() {
    // Start serial objects    
    Serial.begin(9600);
    #ifdef XBEE 
    xbee.setSerial(Serial);
    #endif 

    // Set pin modes
    pinMode(LED, OUTPUT);
    pinMode(CS, OUTPUT);

    // Initialize
    Serial.println("RaggedPi Project Codename Cloves Initializing...");
    
    // Start humidity sensor
    htu.begin();
    
    // Start spi
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV16);
    SPI.setDataMode(SPI_MODE1);
    SPI.setBitOrder(MSBFIRST);    
    
    // Start lightning arrestor
    AS3935.AS3935_DefInit();
    AS3935.AS3935_ManualCal(AS3935_CAPACITANCE, AS3935_OUT, AS3935_DIST_EN);
    attachInterrupt(0, AS3935_ISR, RISING);
    
    // Start barometric pressure sensor
    if (!barometricPressure.begin())   Serial.println("[WARN] Barometric pressure sensor failed to start.");
    
    // Configure weatherstation settings
    weatherStation.setWindMode(MODE, SAMPLE_TIME);

    // Set intial values
    rainfall = 0.0;
}

/**
 * Loop 
 */
void loop() {

    // Check read time
    if (millis - lastReadTime <= READ_TIME) {
        int timer = barometricPressure.startTemperature();
        
        readTemperature();
        readBarometricPressure();
        readAnemometer();
        
        rainfall += weatherStation.get_current_rain_total();

        if (AS3935_ISR_Trig)    doAS3935();         

        #ifdef XBEE
        buildData(0); 
        sendData();
        #endif
        lastReadTime = millis();        
    }
}
