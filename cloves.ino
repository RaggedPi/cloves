/*
  RaggedPi Project
  Arduino 4 "Cloves" - Exterior
  Written by david durost <david.durost@gmail.com>
*/

#include <TimeLib.h>            // Time library
#include <Wire.h>               // One Wire library
#include <DS3231.h>             // Realtime Clock library
#include <Adafruit_ADS1015.h>   // Lightning Detector library
#include <SDL_Weather_80422.h>  // Weather Station library
#include <SPI.h>                // SPI library
#include <SD.h>                 // SD library

//#define DEBUG                 // uncomment for debugger
// Weather Station
#define ANEMPIN 18              // pin
#define ANEMINT 5               // int                 
#define RAINPIN 2               // pin
#define RAININT 1               // int
#define SAMPLETIME 5.0
#define MODE SDL_MODE_SAMPLE // SDL_MODE_DELAY
// Potentiometers
#define POTENTIOMETER1 4
#define POTENTIOMETER2 5

#define LED 13                  // led pin
#define CHIPSELECT 4            // prewired to 10 override if needed
#define ECHO_TO_SERIAL          // debug variable

SDL_Weather_80422 weatherStation(ANEMPIN, RAINPIN, ANEMINT, RAININT, A0, SDL_MODE_INTERNAL_AD);
uint8_t i;
DS3231 RTC(SDA, SCL);
File logfile;                   // log file
float cWindSp;                  // Wind speed
float cWindDir;                 // Wind direction
float cGust;                    // Wind gust speed
float rainfall;                 // Total rainfall

/**
 * Error
 * @param char str
 */
void error(char *str)
{
    Serial.print("error: ");
    Serial.println(str);

    digitalWrite(LED, HIGH);

    while(1);
}

/**
 * Get temp
 * @param  bool celsius
 * @return float
 */
float getTemp(bool celsius = false) {
    if (!celsius) {
        float temp = RTC.getTemp();
        temp = (temp * 1.8)+32;

        return temp;
    }

    return RTC.getTemp();
}

/**
 * Setup
 */
void setup() {
    Serial.begin(9600);
    Serial.println("RaggedPi Project Codename Cloves Initializing.");
    // RTC  
    RTC.begin();
    /* unconnect to set RTC
    RTC.setDate(24,9,2016);
    RTC.setTime(21,14,0);
    */
    if(timeStatus()!= timeSet) {
        Serial.println("Unable to sync with the RTC");
    } else {
        Serial.println("RTC has set the system time");
    }
    // SD
    Serial.println("Initializing SD card...");
    if (!SD.begin(CHIPSELECT)) {
        Serial.println("Card failed, or missing.");
        return;
    }
    Serial.println("Card initialized.");
    // Logging
    char filename[] = "LOG00.CSV";
    for (uint8_t i = 0; i < 100; i++) {
        filename[3] = i/10 + '0';
        filename[4] = i%10 + '0';
        if (!SD.exists(filename)) {
            logfile = SD.open(filename, FILE_WRITE); 
            break;
        }
    }

    if (! logfile) {
        error("logfile creation failed.");
    }

    Serial.print("Logging to: ");
    Serial.println(filename);
    
    Serial.print("Initializing Weather Station");
    weatherStation.setWindMode(MODE, SAMPLETIME);
    rainfall = 0.0;
    for(int x=0; x <= 10; x++) {
        Serial.print(".");
        delay(100);
    }
    Serial.println("\nWeather Station initialized.");
}

/**
 * Loop
 */
void loop() {
    String now = RTC.getDateStr();
    now += "@";
    now += RTC.getTimeStr();

    // Get readings
    cWindSp = weatherStation.current_wind_speed()/1.6;
    cGust = weatherStation.get_wind_gust()/1.6;
    cWindDir = weatherStation.current_wind_direction();
    rainfall = rainfall + weatherStation.get_current_rain_total()/25.4;
    // Log data
    String data = now;
    data += "\tTemperature: ";
    data += getTemp();
    data += "*F";
    data += "\tRainfall: ";
    data += rainfall;
    data += """\tWindspeed: ";
    data += cWindSp;
    data += "MPH\tWind Gust: ";
    data += cGust;
    data += "MPH\tWind Direction: ";
    data += cWindDir;

    logfile.println(data);

    #ifdef ECHO_TO_SERIAL
    Serial.println(data);
    #endif
    delay(60000);

    logfile.close();
}
