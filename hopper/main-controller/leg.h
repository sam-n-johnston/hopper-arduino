#ifndef LEG_H
#define LEG_H

#include "servo.h"
#include "vector.h"
#include <SPI.h>
#include <SoftwareSerial.h>

extern SoftwareSerial DEBUG_SERIAL;

class Leg {
private:
    IServo *servo1;
    IServo *servo2;
    IServo *servo3;
    int footSensorPin = A1;

public:
    Leg(IServo *servo1, IServo *servo2, IServo *servo3);

    void begin();
    void setPosition(float x, float y, float z);
    Vector getFootPosition();
    float getAlphaXInDeg();
    float getAlphaYInDeg();
    void setDesiredAlphaXInDeg(float deg);
    void setDesiredAlphaYInDeg(float deg);
    void pushByFactor(float factor);
    bool isFootTouchingGround();
    void torqueOff();
    void torqueOn();
};

#endif