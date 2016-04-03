/*
  RaggedPi Project
  Arduino 4 "Cloves" - Exterior
  Written by david durost <david.durost@gmail.com>
*/

/* Includes */
#include <TimeLib.h>
#include <Wire.h>
#include <DS3231.h>
#include <Adafruit_ADS1015.h>
#include <SDL_Weather_80422.h>
#include <Servo.h>

/* Defines */
#define DEBUG 1
// Weather Station
#define ANEMPIN 2
#define ANEMINT 5
#define RAINPIN 3
#define RAININT 1
#define SAMPLETIME 5.0
#define MODE SDL_MODE_SAMPLE // SDL_MODE_DELAY
// LDR sensors
#define LDRLT 1 // top left
#define LDRRT 2 // top right
#define LDRLD 3 // bottom left
#define LDRRD 4 // bottom right
// Servos
#define HORIZONTALSERVO 9
#define VERTICALSERVO 10
// Potentiometers
#define POTENTIOMETER1 4
#define POTENTIOMETER2 5

/* Initializations */
SDL_Weather_80422 weatherStation(ANEMPIN, RAINPIN, ANEMINT, RAININT, A0, SDL_MODE_INTERNAL_AD);
uint8_t i;

/* Variables */
// Weather Station
float cWindSp;
float cWindDir;
float cGust;
float rainfall;
// Servos
int hServo = 90;
int vServo = 90;

/**
 * Setup
 */
void setup() {
    Serial.begin(9600);
    Serial.println("RaggedPi Project Codename Cloves Initialized.");

    initializeWeatherStation();
//    initializeServos();
}

/**
 * Loop
 */
void loop() {
    readWeatherStation();
}

/**
 * Initialize servos
 */
void initializeServos() {
    #ifdef DEBUG
    Serial.println("[DEBUG] Entered initializeServos.");
    #endif
//    hServo.attach(HORIZONTALSERVO);
//    vServo.attach(VERTICALSERVO);
}

/**
 * Read LDR array
 */
void readLdrArray() {
    #ifdef DEBUG
    Serial.println("[DEBUG] Entered readLdrArray.");
    #endif
    // Read LDRs
    int lt = analogRead(LDRLT);
    int rt = analogRead(LDRRT);
    int ld = analogRead(LDRLD);
    int rd = analogRead(LDRRD);

    // Read potentiometers
    int dtime = analogRead(POTENTIOMETER1);
    int tolerance = analogRead(POTENTIOMETER2)/4;

    // Calculate average values
    int avt = (lt + rt) / 2; // top
    int avd = (ld + rd) / 2; // down
    int avl = (lt + ld) / 2; // left
    int avr = (rt + rd) / 2; // right

    // Calculate differences
    int dvert = avt - avd; // up and down
    int dhoriz = avl - avr; // left and rigt

    // Move servos as needed
    moveServo(&vServo, dvert, avt, avd, tolerance, VERTICALSERVO);
    moveServo(&hServo, dhoriz, avr, avl, tolerance, HORIZONTALSERVO);
    delay(dtime);


}

/**
 * Move servo
 * @param  int  servo         
 * @param  int  difference    
 * @param  int  avgA          
 * @param  int  avgB          
 * @param  int  tolerance     
 * @param  bool axis          
  */
void moveServo(int servo, int difference, int avgA, int avgB, int tolerance, bool axis = 0) {
    #ifdef DEBUG
    Serial.println("[DEBUG] Entered moveServo.");
    Serial.print("Passed Param: servo = ");
    Serial.print(servo);
    Serial.print("\nPassed Param: difference = ");
    Serial.print(difference);
    Serial.print("\nPassed Param: avgA = ");
    Serial.print(avgA);
    Serial.print("\nPassed Param: avgB = ");
    Serial.print(avgB);
    Serial.print("\nPassed Param: Tolerance = ");
    Serial.print(tolerance);
    Serial.print("\nPassed Param: axis = ");
    Serial.print(axis);
    Serial.print("\n");
    #endif
    
    if (-1*tolerance > difference || difference > tolerance) {
        if (avgA > avgB) {
            servo = ++vServo;
            if (servo > 180) {
                servo = 180;
            }
        } else if (avgA < avgB) {
            servo= --servo;
            if (servo < 0) {
                servo = 0;
            }
        }

        if (axis) {
            vertical.write(servo);
        } else {
            horizontal.write(servo);
        }
    }
}

/**
 * Initialize weather station
 */
void initializeWeatherStation() {
    #ifdef DEBUG
    Serial.println("[DEBUG] Entered initializeWeatherStation.");
    #endif
    Serial.println("Initializing Weather Station...");
    weatherStation.setWindMode(MODE, SAMPLETIME);
    rainfall = 0.0;
    delay(1000);
    Serial.println("Weather Station initialized.");
}

/**
 * Read weather station
 */
void readWeatherStation() {
    #ifdef DEBUG
    Serial.println("[DEBUG] Entered readWeatherStation.");
    #endif
    // Get current wind speed
    cWindSp = weatherStation.current_wind_speed()/1.6;
    
    // Get current wind gust
    cGust = weatherStation.get_wind_gust()/1.6;

    // Get current wind direction
    cWindDir = weatherStation.current_wind_direction();
    
    // Get total rainfall
    rainfall = rainfall + weatherStation.get_current_rain_total()/25.4;

    // Output
    Serial.print("Rainfall: ");
    Serial.print(rainfall);
    Serial.print("""\tWindspeed: ");
    Serial.print(cWindSp);
    Serial.print("MPH\tWind Gust: ");
    Serial.print(cGust);
    Serial.print("MPH\tWind Direction: ");
    Serial.print(cWindDir);
    Serial.print("\n");
  
    delay(1000);
}