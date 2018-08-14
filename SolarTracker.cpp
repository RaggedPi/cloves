#include "SolarTracker.h"
//#include <Servo.h>

float voltage = 0.00;
int tl;
int tr;
int bl;
int br; 
int avgTop;
int avgBot;
int avgLeft;
int avgRight;
Servo horizontal;
Servo vertical;

/**
 * Constructor
 */
SolarTracker::SolarTracker() {}
void SolarTracker::begin() {
    pinMode(LDR1, INPUT);
    pinMode(LDR2, INPUT);
    pinMode(LDR3, INPUT);
    pinMode(LDR4, INPUT);

    horizontal.attach(H_SERVO);
    //moveServo(horizontal, 90);

    vertical.attach(V_SERVO);
    //moveServo(vertical, 90);
}

/**
 * Move servo
 * @param  Servo servo
 * @param  int position
 */
void SolarTracker::moveServo(Servo servo, int position) {
    servo.write(position);
}

/**
 * Caclulate average LDR readings
 */
void SolarTracker::calculateAverages(void) {
    avgTop = (tl + tr) / 2;
    avgBot = (bl + br) / 2;
    avgLeft = (tl + bl) / 2;
    avgRight = (tr + br) / 2;
}

/**
 * Calculate servo position and move servo
 */
void SolarTracker::calculatePosition(bool moveToPos) {
    // Read sensors
    tl = analogRead(LDR1);
    tr = analogRead(LDR2);
    bl = analogRead(LDR3);
    br = analogRead(LDR4);

    // Calculate averages
    SolarTracker::calculateAverages();

    // Read servos
    int h = horizontal.read();
    int v = vertical.read();

    if (moveToPos) {
        // Compare avg vs servo position
        // Vertical axis
        if (avgTop < avgBot) {
            if (MAX_DEG <= v) {
                moveServo(vertical, ++v);
            }
            delay(10);
        } else if (avgBot < avgTop) {
            if (MIN_DEG >= v) {
                moveServo(vertical, --v);
            }
            delay(10);
        } else {
            // no need to move..
        }
        // Horizontal axis
        if (avgLeft < avgRight) {
            if (MAX_DEG <= h) {
                moveServo(horizontal, ++h);
            }
            delay(10);
        } else if (avgRight < avgLeft) {
            if (MIN_DEG >= h) {
                moveServo(horizontal, --h);
            }
            delay(10);
        } else {
            // no need to move..
        }
        delay(50);
    }
}

/**
 * Read panel voltage
 * @todo read voltages
 * @return float
 */
float SolarTracker::readPanelVoltage(void) {
    return 48.00;
}