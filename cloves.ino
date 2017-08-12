/*
  RaggedPi Project
  Arduino "Cloves" - Weatherstation
  Written by david durost <david.durost@gmail.com>
*/
/* Includes */
#include <SoftwareSerial.h>             // Serial library
#include <Wire.h>                       // One Wire library
#include <SDL_Weather_80422.h>          // Weather Station library
#include <Adafruit_ADS1015.h>           // 
#include <Adafruit_HTU21DF.h>           // Humidity library
#include <SPI.h>                        // SPI library
#include <PWFusion_AS3935.h>            // Lightning Detection library
#include <SFE_BMP180.h>                 // Barometric Pressure library
#include <Time.h>                       // Time library

/* Misc Constants */
#define SDA 20                          // sda
#define SDL 21                          // sdl
#define CS 53                           // chipselect
#define LED 13                          // led pin
#define SI 4                            // select input
// weatherstation
#define ANEM_PIN 18                     // digital pin
#define ANEM_INT 5                      // int                 
#define RAIN_PIN 7                      // digital pin
#define RAIN_INT 1                      // int
#define SAMPLE_TIME 5.0                 // float
#define MODE SDL_MODE_SAMPLE            //
#define ALTITUDE 1655.0                 // Altitude in meters
#define ALTITUDE_F (_m2ft(ALTITUDE))    // Altitude in feet (m*3.28084)
// potentiometers
#define POTENTIOMETER1_PIN 4            // analog pin
#define POTENTIOMETER2_PIN 5            // analog pin
// lightning arrestor
#define AS3935PIN 2                     // analog pin AS3935 IRQ
#define AS3935_IN 0                     //
#define AS3935_OUT 1                    //
#define AS3936_DIST_DIS 0               //
#define AS3935_DIST_EN 1                //
#define AS3935_CAPACITANCE 104          // on board

/* Objects */
void AS3935_ISR();                      // initalize arrestor
SDL_Weather_80422 weatherStation(
    ANEM_PIN, RAIN_PIN,
    ANEM_INT, RAIN_INT,
    A0, SDL_MODE_INTERNAL_AD);          // weatherstation
Adafruit_HTU21DF htu = Adafruit_HTU21DF(); // humidity sensor
PWF_AS3935 AS3935(CS, AS3935PIN, SI);   // lightning arrestor 
SFE_BMP180 pressure;                    // pressure object

/* Variables */
float cWindSp;                          // current wind speed
float cWindDir;                         // current wind direction
float cGust;                            // current wind gust speed
float rainfall;                         // total rainfall
double Bmp;                             // pressure placeholder
double Temp;                            // temperature placeholder
volatile int AS3935_ISR_Trig = 0;       // interrupt
char* msg;                              // reused variable
char status;                            // reused variable

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
 * Get temperature string
 * @param  bool fahrenheit return temperature in celsius or fahrenheit
 * @param  bool longform return temperature string with a descriptor
 * @return char*
 */
char* getTemperatureStr(bool fahrenheit=false, bool longform=false) {
    status = pressure.startTemperature(); // returns ms read time
    if (0 != status) {
        delay(5);
        status = pressure.getTemperature(Temp); // return 1 or 0, reading stored in Temp
        if (0 != status) {
            char *tmp;
            char cf = 'C';
            double t = Temp;

            if (fahrenheit) {
                cf = 'F';
                t = ((9.0/5.0)*Temp)+32.0;
            }

            sprintf(tmp, "%d %c %c", (int)t, 0176, cf); 
            if (longform)   sprintf(msg, "temperature: %s", tmp);
            else    sprintf(msg, "%s", tmp);
            return msg;
        } else  Serial.println("[Warning] Failed to read temperature.");
    } else  Serial.println("[Warning] Failed to start temperature sensor.");
    return 0;
}

/**
 * Get barometric pressure string
 * @param  bool absOrSea true for sealevel pressure, false for current pressure
 * @param  bool mbOrInHg true for inHg, false for mb
 * @param  bool longform true for descriptor
 * @return char*
 */
char* getPressureStr(bool absOrSea=false, bool mbOrInHg=false, bool longform=false) {
    if (0 == Temp)  status = pressure.getTemperature(Temp);
    status = pressure. startPressure(3);     // [0-3] resolution, returns ms read time
    if (0 != status) {
        delay(status);
        status = pressure.getPressure(Bmp,Temp);    // returns 0 or 1
        if (0 != status) {
            char *mb = "mb";
            double p = Bmp;
            if (absOrSea)   p = pressure.sealevel(Bmp, ALTITUDE);
            if (mbOrInHg) {
                mb = "inHg";
                p *= 0.0295333727;
            }
            sprintf(msg, "%d %s", p, mb);
            if (longform) {
                status = '\0';
                if (absOrSea)   status = 'sealevel ';
                char *msg2 = msg;
                sprintf(msg, "%sbarometric pressure: %s", status, msg2);
            }
            return msg;
        } else  Serial.println("[Warning] Failed to get pressure.");
    }   else Serial.println("[Warning] Failed to start pressure.");
    return 0;
}

/**
 * Get wind direction string
 * @param  float heading
 * @param  bool  longform
 * @return char*
 */
char* getWindDirectionStr(float heading, bool longform=false) {
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
 * AS3935 interrupt handler
 */
void AS3935_ISR() {
    AS3935_ISR_Trig = 1;
}

void setup() {
    Serial.begin(9600);
    
    /* Set pins ****************************************************************/
    pinMode(LED, OUTPUT);
    pinMode(CS, OUTPUT);

    Serial.println("RaggedPi Project Codename Cloves Initializing...");
    
    /* Humidity sensor *********************************************************/
    Serial.print("Initializing humidity sensor...");
    if (!htu.begin()) {
        Serial.println("[Fail]");
    } else  Serial.println("[OK]");
    
    /* SPI ********************************************************************/
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV16);
    SPI.setDataMode(SPI_MODE1);
    SPI.setBitOrder(MSBFIRST);    
    
    /* AS3935 Lightning Detection *********************************************/
    Serial.print("Intializing lightning detection...");
    AS3935.AS3935_DefInit();
    AS3935.AS3935_ManualCal(AS3935_CAPACITANCE, AS3935_IN, AS3935_DIST_EN);
    attachInterrupt(0, AS3935_ISR, RISING);
    Serial.println("[OK]");

    /* BMP180 Pressure Sensor *************************************************/
    Serial.print("Initializing barometric pressure sensor...");
    if (pressure.begin())   Serial.println("[OK]");
    else    Serial.println("[Fail]");

    /* Weather Station ********************************************************/
    Serial.print("Initializing Weather Station...");
    weatherStation.setWindMode(MODE, SAMPLE_TIME);
    rainfall = 0.0;
    Serial.println("[OK]");

    /* Initialized ************************************************************/
    Serial.println("System initialized.");
}

/**
 * Loop
 */
void loop() {
    /* Pressure and temperature ************************************************/
    status = pressure.startTemperature();
    char* temperature = getTemperatureStr(true, true);
    char* barometric = getPressureStr(false, false, true);
    
    /* Windspeed and rainfall **************************************************/
    cWindSp = weatherStation.current_wind_speed()/1.6;
    cGust = weatherStation.get_wind_gust()/1.6;
    cWindDir = weatherStation.current_wind_direction();
    rainfall = rainfall + weatherStation.get_current_rain_total()/25.4;    

    /* Lightning detection *****************************************************/
    if (AS3935_ISR_Trig) {
        AS3935_ISR_Trig = 0; // reset flag
        uint8_t src = AS3935.AS3935_GetInterruptSrc(); // get source
        switch (src) {
            case 0:
                Serial.println("[ERROR] AS3935 interrupt source result not expected.");
                break;
            case 1: {
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

    /* Output data *************************************************************/
    sprintf(
        msg,
        "[%s][Humidity: %s%%][%s][Windspeed: %dmph %s][Wind gust: %dmph][Rainfall: %d in]",
        temperature,
        htu.readHumidity(),
        barometric,
        cWindSp,
        getWindDirectionStr(cWindDir),
        cGust,
        rainfall);
    Serial.println(msg);
    delay(60000);
    
    #ifdef LOG
    logfile.close();
    #endif
    
    #ifdef DEBUG
    Serial.print("msg: "); Serial.println(msg);
    Serial.println("Exit loop()");
    #endif
}
    