#ifndef SOLARTRACKER_H
#define SOLARTRACKER_H

#if ARDUINO >= 100
    #include "Arduino.h"
#else
    #include "WProgram.h"
#endif
#include <Servo.h>

#define H_SERVO 9       // digital pin
#define V_SERVO 10      // digital pin

#define LDR1 A1         // analog pin
#define LDR2 A2         // analog pin
#define LDR3 A3         // analog pin
#define LDR4 A4         // analog pin

#define MIN_DEG 0       // minimum angle
#define MAX_DEG 180     // maximum angle

class SolarTracker
{
    public:
        SolarTracker();
        void begin(void);
        void moveServo(Servo servo, int position);
        void calculatePosition(bool moveToPos=true);
        float readPanelVoltage(void);
    private:
        void calculateAverages(void);        
};
#endif
