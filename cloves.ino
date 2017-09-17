/*
  RaggedPi Project
  Arduino "Cloves" - Weatherstation
  Written by david durost <david.durost@gmail.com>
*/
/* Includes */
#include <SoftwareSerial.h>                 // Serial library
#include <Wire.h>                           // One Wire library
#include <SDL_Weather_80422.h>              // Weather Station library
#include <Adafruit_ADS1015.h>               // 
#include <Adafruit_HTU21DF.h>               // Humidity library
#include <SPI.h>                            // SPI library
#include <PWFusion_AS3935.h>                // Lightning Detection library
#include <SFE_BMP180.h>                     // Barometric Pressure library
#include <Time.h>                           // Time library
#include <Servo.h>                          // Server library
#include <SolarTracker.h>                   // Solar Tracker library

/* Misc Constants */
#define SDA 20                              // sda
#define SDL 21                              // sdl
#define CS 53                               // chipselect
#define LED 13                              // led pin
#define SI 4                                // select input
// Barometric pressure
#define ABS 0
#define SEA 1
#define ABS_HG 2
#define SEA_HG 3
#define MB2INHG 0.0295333727
// Weatherstation
#define ANEM_PIN 18                         // digital pin
#define ANEM_INT 5                          // int                 
#define RAIN_PIN 2                          // digital pin
#define RAIN_INT 1                          // int
#define SAMPLE_TIME 5.0                     // float
#define MODE SDL_MODE_SAMPLE                // sample type
#define ALTITUDE 145.0                     // Altitude in meters
#define ALTITUDE_F (_m2ft(ALTITUDE))        // Altitude in feet (m*3.28084)
// Lightning arrestor
#define AS3935_PIN 2                        // analog pin AS3935 IRQ
#define AS3935_IN 0                         //
#define AS3935_OUT 1                        //
#define AS3936_DIST_DIS 0                   //
#define AS3935_DIST_EN 1                    //
#define AS3935_CAPACITANCE 104              // on board
// Solar
#define BATTERY_PIN A0                      // analog pin
/* Uncomment below to modify the default settings */
// #define LDR1 A1                             // analog pin [top left]
// #define LDR2 A2                             // analog pin [top right]
// #define LDR3 A3                             // analog pin [bottom left]
// #define LDR4 A4                             // analog pin [bottom right]
// #define H_SERVO 9                           // digital pin
// #define V_SERVO 10                          // digital pin
// MIN_DEG 0                                   // minimum angle
// MAX_DEG 180                                 // maximum angle

void AS3935_ISR();

SDL_Weather_80422 weatherStation(
    ANEM_PIN, RAIN_PIN, ANEM_INT, RAIN_INT, A0, SDL_MODE_INTERNAL_AD
);
Adafruit_HTU21DF htu = Adafruit_HTU21DF();
PWF_AS3935 AS3935(CS, AS3935_PIN, SI);
SFE_BMP180 barometricPressure;
SolarTracker solar = SolarTracker();
;
int batteryValue;
double tempC,
       tempF, 
       p, 
       bmp[] = { 0.0, 0.0, 0.0, 0.0 };
float curWindGust, 
      curWindDirection, 
      curWindSpeed, 
      rainfall, 
      batteryVoltage;
volatile int AS3935_ISR_Trig = 0;

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
 * AS3935 interrupt handler
 */
void AS3935_ISR() {
    AS3935_ISR_Trig = 1;
}

/**
 * Read temperature
 */
void readTemperature() {
    if (0 != barometricPressure.startTemperature()) {
        delay(5);
        if (0 != barometricPressure.getTemperature(tempC)) {
            tempF = ((9.0/5.0)*tempC)+32.0;
        } else Serial.println("[WARN] Could not read temperature");
    } else Serial.println("[WARN] Could not start temperature sensor.");
}

/**
 * Print temperature
 * @param  int cf_c_f 0: both (default), 1: celsius, 2: fahrenheit
 */
void printTemperature(int cf_c_f = 0) {
    if (0 == cf_c_f || 1 == cf_c_f) {
        Serial.print((int)tempC);
        Serial.print("C ");
    }
    if (0 == cf_c_f || 2 == cf_c_f) {
        Serial.print((int)tempF);
        Serial.print("F ");
    }
}

/**
 * Read barometric pressure
 */
void readBarometricPressure() {
    int p;
    if(0 == tempC)  p = barometricPressure.getTemperature(tempC);
    p = barometricPressure.startPressure(3); // [0-3] resolution, return ms read time
    if (0 != p) {
        delay(p);
        if (0 != barometricPressure.getPressure(bmp[ABS], tempC)) {
            bmp[SEA] = barometricPressure.sealevel(bmp[ABS], ALTITUDE);
            bmp[ABS_HG] = bmp[ABS] * MB2INHG;
            bmp[SEA_HG] = bmp[SEA] * MB2INHG; 
        } else Serial.println("[WARN] Could not get pressure.");
    } else Serial.println("[WARN] Could not start pressure sensor.");
}

/**
 * Print barometric pressure
 * @param  bool absOrSea     
 * @param   bool mbOrInHg 
 */
void printBarometricPressure(bool absOrSea = false, bool mbOrInHg = false) {
    int index = 0;
    if (absOrSea)   index++;
    if (mbOrInHg)   index += 2;

    Serial.print(bmp[index]);
    if (mbOrInHg)   Serial.print("InHg");
    else    Serial.print("mb");
}

/**
 * Get wind direction string
 * @param  float heading
 * @param  bool  longform
 * @return char*
 */
char* getWindDirectionStr(float heading, bool longform = false) {
    char* directions[] = {
        "n", "nne", "ne", "ene", 
        "e", "ese", "se", "sse",
        "s", "ssw", "sw", "wsw",
        "w", "wnw", "nw", "nnw",
        "n"
    }; 
    char* directions_long[] = {
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
    int index = round((round(heading) % 360) / 22.5)+1;
    if (longform)   return directions_long[index];
    return directions[index];
}

/**
 * Read battery level
 */
void readBatteryData() {
    batteryValue = analogRead(BATTERY_PIN);
    batteryVoltage = (float(batteryValue)*5)/1023*2;
}

/**
 * Print battery value or voltage
 * @param  {Boolean} bool default: true (voltage)
 */
void printBatteryData(bool voltage=false) {
    if (!voltage)    Serial.print(batteryValue);
    else    Serial.print(batteryVoltage);
}

/*
 * Print weather station data
 */
void printData() {
    Serial.print("[Bat: ");
    printBatteryData();
    Serial.print("V][");
    Serial.print((int)tempF);
    Serial.print("F][Humidity: ");
    Serial.print(htu.readHumidity());
    Serial.print("%][Pressure: ");
    Serial.print(bmp[ABS_HG]);
    Serial.print("InHg][Wind speed: ");
    Serial.print(curWindSpeed);
    Serial.print("mph ");
    Serial.print(getWindDirectionStr(curWindDirection));
    Serial.print("][Wind gust: ");
    Serial.print(curWindGust);
    Serial.print("mph][Total rainfall: ");
    Serial.print(rainfall);
    Serial.print("in]");
}

/**
 * Setup
 */
void setup() {
    Serial.begin(9600);
    
    pinMode(LED, OUTPUT);
    pinMode(CS, OUTPUT);

    Serial.println("RaggedPi Project Codename Cloves Initializing...");
    
    htu.begin();
    
    solar.begin();

    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV16);
    SPI.setDataMode(SPI_MODE1);
    SPI.setBitOrder(MSBFIRST);    
    
    AS3935.AS3935_DefInit();
    AS3935.AS3935_ManualCal(AS3935_CAPACITANCE, AS3935_OUT, AS3935_DIST_EN);
    attachInterrupt(0, AS3935_ISR, RISING);
    
    if (!barometricPressure.begin())   Serial.println("[WARN] Barometric pressure sensor failed to start.");
    
    weatherStation.setWindMode(MODE, SAMPLE_TIME);
    rainfall = 0.0;
    tempC = 0.0;
    tempF = 0.0;
    p = 0.0;

    Serial.println("System initialized.");
}

/**
 * Loop 
 */
void loop() {
    Serial.println("loop.");
    int x = barometricPressure.startTemperature();
    readTemperature();
    readBarometricPressure();
    curWindSpeed = weatherStation.current_wind_speed()/1.6;
    curWindGust = weatherStation.get_wind_gust()/1.6;
    curWindDirection = weatherStation.current_wind_direction();
    rainfall += weatherStation.get_current_rain_total();

    if (AS3935_ISR_Trig) {
        AS3935_ISR_Trig = 0; // reset flag
        switch (AS3935.AS3935_GetInterruptSrc()) {
            case 0:
                Serial.println("[ERROR] AS3935 interrupt source result not expected.");
                break;
            case 1: {
                char* msg;
                uint8_t distance = AS3935.AS3935_GetLightningDistKm();
                // 1 - near, 63 - distant
                if (1 == distance)  Serial.println("Lightning overhead!");
                else if (63 == distance)    Serial.println("Out of range lightning detected.");
                else if (63 > distance && 1 < distance) {
                    sprintf(msg, "Lightning detected! (%dkm)", distance);
                    Serial.println(msg);
                }
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

    solar.calculatePosition(true);

    printData();
    delay(60000);
}