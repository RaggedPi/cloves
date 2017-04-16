/*
  RaggedPi Project
  Arduino 4 "Cloves" - Exterior [Mega]
  Written by david durost <david.durost@gmail.com>
*/
/*******************************************************************************
** INCLUDES ********************************************************************
*******************************************************************************/
#include <SoftwareSerial.h>
#include <TimeLib.h>            // Time library
#include <Wire.h>               // One Wire library
#include <DS3231.h>             // Realtime Clock library
#include <SDL_Weather_80422.h>  // Weather Station library
#include <Adafruit_ADS1015.h>   // 
#include <Adafruit_HTU21DF.h>   // Humidity library
#include <SPI.h>                // SPI library
#include <SD.h>                 // SD library
#include <PWFusion_AS3935.h>    // Lightning Detection library
#include <SFE_BMP180.h>         // Barometric Pressure library
/*******************************************************************************
** DEBUGGING *******************************************************************
*******************************************************************************/
#define DEBUG                   // uncomment for debugger
//#define XBEE                    // comment to disable xbee transmission
//#define LOG                     // comment to disable datalogging
#define SERIAL                  // comment to mute serial output
/*******************************************************************************
** MISC CONSTANTS **************************************************************
*******************************************************************************/
#define SDA 20                  // sda
#define SDL 21                  // sdl
#define CS 53                   // chipselect 10 - uno, 53 - mega
#define LED 13                  // led pin
#define SI 4                    // selectinput
/*******************************************************************************
** WEATHER STATION *************************************************************
*******************************************************************************/
#define ANEMPIN 18               // digital pin
#define ANEMINT 5               // int                 
#define RAINPIN 7               // digital pin
#define RAININT 1               // int
#define SAMPLETIME 5.0          // float
#define MODE SDL_MODE_SAMPLE //SDL_MODE_DELAY
#define ALTITUDE 1655.0         // Altitude in meters
#define ALTITUDE_F (_m2ft(ALTITUDE)) //5429         // Altitude in feet (m*3.28084)
/*******************************************************************************
** POTENTIOMETERS **************************************************************
*******************************************************************************/
#define POTENTIOMETER1 4        // analog pin
#define POTENTIOMETER2 5        // analog pin
/*******************************************************************************
** LIGHTNING ARRESTOR **********************************************************
*******************************************************************************/
#define AS3935PIN 2             // analog pin AS3935 IRQ
#define AS3935_IN 0             //
#define AS3935_OUT 1            //
#define AS3936_DIST_DIS 0       //
#define AS3935_DIST_EN 1        //
#define AS3935_CAPACITANCE 104  // on board
/*******************************************************************************
** OBJECTS *********************************************************************
*******************************************************************************/
void AS3935_ISR();              // initalize arrestor
SDL_Weather_80422 weatherStation(
    ANEMPIN, RAINPIN,
    ANEMINT, RAININT,
    A0, SDL_MODE_INTERNAL_AD);  // weatherstation
DS3231 RTC(SDA, SCL);           // realtime Clock
Adafruit_HTU21DF htu = Adafruit_HTU21DF(); // humidity sensor
PWF_AS3935 AS3935(CS, AS3935PIN, SI); // lightning arrestor 
SFE_BMP180 pressure;            // pressure object
#ifdef XBEE
SoftwareSerial XBee(SDA, SDL);  // xbee2 object
#endif
/*******************************************************************************
** VARIABLES *******************************************************************
*******************************************************************************/
File logfile;                   // log file
float cWindSp;                  // current wind speed
float cWindDir;                 // current wind direction
float cGust;                    // current wind gust speed
float rainfall;                 // total rainfall
double Bmp;                     // pressure placeholder
double Temp;                    // temperature placeholder
char* msg;                      // reused variable
char status;                   // reused variable
volatile int AS3935_ISR_Trig = 0; // interrupt
/*******************************************************************************
** UTILITY METHODS *************************************************************
*******************************************************************************/
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
 * Print
 * @param  char *str
 * @param  bool xbee
 * @param  bool serial
 */
void print(char *str, bool log=true, bool xbee=true, bool serial=true) {
    if (serial) {
        #ifdef SERIAL
        Serial.print(str);
        #endif
    }
    if (xbee) {
        #ifdef XBEE
        XBee.print(str);
        #endif
    }
    if (log) {
        #ifdef LOG
        logfile.print(str);
        #endif
    }
}

/**
 * Print line
 * @param  char *str
 * @param  bool xbee
 * @param  bool serial
 */
void println(char *str=NULL, bool log=true, bool xbee=true, bool serial=true) {
    print(str, log, xbee, serial);
    print("\n", log, xbee, serial);
}

/**
 * Blink LED
 * @param  uint8_t pin
 * @param  int     max
 * @param  int     interval ms
 */
void blinkLED(uint8_t pin, int max=10, int interval=500) {
    for (int i=0; i < max; i++) {
        digitalWrite(pin, HIGH);
        delay(interval);
        digitalWrite(pin, LOW);
        if (max > (i + 1))    delay(interval);
    }
}

/**
 * Error
 * @param  char *str
 * @param  bool halt default: true
 */
void error(char* str, bool halt=true) {
    char *str2;
    blinkLED(LED);
    sprintf(str2, "[ERROR] %s", str);
    println(str2);
    if (halt)   while(1);
}

/**
 * Warning
 * @param  char *str
 */
void warning(char *str) {
    char *str2;
    blinkLED(LED, 5);
    sprintf(str2, "[WARNING] %s", str);
    println(str2);
}

/**
 * Dot dot dot
 * @param  uint8_t dot_total
 * @param  char dot
 * @param  uint8_t ms_per_dot
 */
void dotdotdot(uint8_t dot_total=10, char* dot=".", uint8_t ms_per_dot=100, bool newline=false) {
    for (int x = 0; x < dot_total; x++) {
        print(dot, false);
        delay(ms_per_dot);
    }
}

/**
 * Get temperature string
 * @param  bool fahrenheit return temperature in celsius or fahrenheit
 * @param  bool longform return temperature string with a descriptor
 * @return char*
 */
char* getTemperatureStr(bool fahrenheit=false, bool longform=false) {
    #ifdef DEBUG
    Serial.println("Enter getTemperatureStr()");
    Serial.print("fahrenheit: "); Serial.println(fahrenheit);
    Serial.print("longform: "); Serial.println(longform);
    #endif
    status = pressure.startTemperature(); // returns ms read time
    #ifdef DEBUG
    Serial.print("status: "); Serial.println(status);
    #endif
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

            #ifdef DEBUG
            Serial.print("cf: "); Serial.println(cf);
            //Serial.print("t: "); Serial.println(t);
            #endif

            sprintf(tmp, "%d %c %c", (int)t, 0176, cf); 
            #ifdef DEBUG
            Serial.print("tmp: "); Serial.println(tmp);
            #endif  
            if (longform)   sprintf(msg, "temperature: %s", tmp);
            else    sprintf(msg, "%s", tmp);
            return msg;
        } else  warning("Failed to read temperature.");
    } else  warning("Failed to start temperature sensor.");
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
        } else  warning("Failed to get pressure.");
    }   else warning("Failed to start pressure.");
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

/*******************************************************************************
** APPLICATION *****************************************************************
*******************************************************************************/
/**
 * Setup
 */
void setup() {
    Serial.begin(9600);
    #ifdef XBEE
    XBee.begin(9600);
    #endif
    
    print("RaggedPi Project Codename Cloves Initializing");
    dotdotdot();
    println();  
    
    /* RTC ********************************************************************/ 
    RTC.begin();
    // print("Initializing RTC");
    // dotdotdot(5);
    // /* uncomment to set RTC 
    RTC.setDate(07,4,2017);
    RTC.setTime(23,20,0);
    // */
  
    // if(!RTC.begin()) {
    //     warning("Failed");
    // } else {
    //     println("Initialized.");
    // }  
    /* Humidity ***************************************************************/
    print("Initializing humidity sensor");
    dotdotdot(5);
    if (!htu.begin()) {
        println("Failed.");
        warning("Failed to initialize htu210 sensor.");
    } else  println("Intialized.");
    
    /* SPI ********************************************************************/
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV16);
    SPI.setDataMode(SPI_MODE1);
    SPI.setBitOrder(MSBFIRST);    
    
    /* SD *********************************************************************/
    // print("Initializing SD card");
    // dotdotdot(10, ".", 100, true);
    // if (!SD.begin(CS))  {
    //     #undef LOG
    //     error("SD card failed or is missing");
    // }
    // println("Initialized.");    
    
    /* Logging ****************************************************************/
    #ifdef LOG
    print("Initializing system logging");
    dotdotdot(5);
    char filename[] = "LOG00.csv";
    for (uint8_t i = 0; i < 100; i++) {
        filename[3] = i / 10 + '0';
        filename[4] = i % 10 + '0';
        if (!SD.exists(filename)) {
            logfile = SD.open(filename, FILE_WRITE);
            break;
        }
    }
    if (!logfile) {
        println("Failed.", false);
        error("Logfile creation failed. System will continue, logging will be disabled.", false);
    } else {
        sprintf(msg, "Intitialized.\nLogfile: %s", filename);
        println(msg);
    }
    #endif

    /* AS3935 Lightning Detection *********************************************/
    print("Intializing lightning detection");
    dotdotdot(5);
    AS3935.AS3935_DefInit();
    AS3935.AS3935_ManualCal(AS3935_CAPACITANCE, AS3935_IN, AS3935_DIST_EN);
    attachInterrupt(0, AS3935_ISR, RISING);
    println("Initialized.");    

    /* BMP180 Pressure Sensor *************************************************/
    print("Initializing barometric pressure sensor");
    dotdotdot(5);
    if (pressure.begin())   println("Initialized.");
    else    println("Failed.");

    /* Weather Station ********************************************************/
    print("Initializing Weather Station");
    dotdotdot(5);
    weatherStation.setWindMode(MODE, SAMPLETIME);
    rainfall = 0.0;
    println("Initialized.");

    /* Misc *******************************************************************/
    pinMode(LED, OUTPUT);
    pinMode(CS, OUTPUT);

    /* Initialized ************************************************************/
    println("System initialized.");
    #ifdef DEBUG
    println("Exit setup()");
    #endif
}

/**
 * Loop
 */
void loop() {
    #ifdef DEBUG
    Serial.println("Enter loop()");
    #endif
    status = pressure.startTemperature();

    #ifdef DEBUG
    Serial.println("[Generate strings]");
    #endif
    /* Generate strings *******************************************************/
    char *now, *dayStr, *timeStr;
    //dayStr = RTC.getDateStr();
    //timeStr = RTC.getTimeStr();
    //sprintf(now, "%s @ %s", dayStr, timeStr);
    #ifdef DEBUG
    //Serial.println(now);
    #endif
    char* temperature = getTemperatureStr(true, true);
    char* barometric = getPressureStr(false, false, true);
    #ifdef DEBUG
    //Serial.print("now: "); Serial.println(now);
    Serial.print("temperature: "); println(temperature);
    Serial.print("barometric"); println(barometric);
    #endif

    #ifdef DEBUG
    Serial.println("[Obtain readings");
    #endif
    /* Get windspeed and rainfall readings ************************************/
    cWindSp = weatherStation.current_wind_speed()/1.6;
    cGust = weatherStation.get_wind_gust()/1.6;
    cWindDir = weatherStation.current_wind_direction();
    rainfall = rainfall + weatherStation.get_current_rain_total()/25.4;    

    #ifdef DEBUG
    Serial.print("cWindsp: "); Serial.println(cWindSp); 
    Serial.print("cGust: "); Serial.println(cGust); 
    Serial.print("cWindDir: "); Serial.println(cWindDir);
    Serial.print("rain: "); Serial.println(rainfall);
    #endif

    /* Lightning detection ****************************************************/
    if (AS3935_ISR_Trig) {
        #ifdef DEBUG
        Serial.println("[Lightning Arrester trigggered");
        #endif
        AS3935_ISR_Trig = 0; // reset flag
        uint8_t src = AS3935.AS3935_GetInterruptSrc(); // get source
        switch (src) {
            case 0:
                error("AS3935 interrupt source result not expected.");
                break;
            case 1: {
                uint8_t distance = AS3935.AS3935_GetLightningDistKm();
                // 1 - near, 63 - distant
                if (1 == distance)  println("Lightning overhead!");
                else if (63 == distance)    println("Out of range lightning detected.");
                else if (63 > distance && 1 < distance) {
                    sprintf(msg, "Lightning detected! (%dkm)", distance);
                    println(msg);
                }
                break;
            }
            case 2:
                error("AS3935 Disturber detected");
                break;
            case 3:
                error("AS3935 Noise level too high.");
                break;    
            default:
                break;
        }        
        #ifdef DEBUG
        AS3935.AS3935_PrintAllRegs();
        #endif
    }

    #ifdef DEBUG
    Serial.println("[Log data]");
    #endif

    /* Log data ***************************************************************/
    sprintf(
        msg,
        "[%s][%s][Humidity: %s%%][%s][Windspeed: %dmph %s][Wind gust: %dmph][Rainfall: %d in]",
        now,
        temperature,
        htu.readHumidity(),
        barometric,
        cWindSp,
        getWindDirectionStr(cWindDir),
        cGust,
        rainfall);
    println(msg);
    delay(60000);
    
    #ifdef LOG
    logfile.close();
    #endif
    
    #ifdef DEBUG
    Serial.print("msg: "); Serial.println(msg);
    Serial.println("Exit loop()");
    #endif
}
