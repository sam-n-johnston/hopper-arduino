#ifndef LEG_H
#define LEG_H

#include "servo.h"
#include "vector.h"
#include <SPI.h>
#include <SoftwareSerial.h>

const float degToRad = 0.0174533;

class Leg {
private:
    Servo *servo1;
    Servo *servo2;
    Servo *servo3;
    int footSensorPin = 28;
    float footLengthInMM = 45.0;
    float pushFactor = 1.0;
    float goalFootExtension = 0;
    float goalX = 0;
    float goalY = 0;
    float goalZ = -75; 
public:
    Leg(Servo *servo1, Servo *servo2, Servo *servo3);

    void begin();
    void setFootPosition(Vector position);
    void setFootPosition(float x, float y, float z);
    Vector getFootPosition();
    float getAlphaXInDeg();
    float getAlphaYInDeg();
    void setPushFactor(float factor);
    void setDesiredAlphaXYInDeg(float degX, float degY);
    void setDesiredAlphaXYInDeg(
        float degX, float degY, float alphaXDeg, float alphaYDeg);
    bool isFootTouchingGround();
    void torqueOff();
    void torqueOn();
};

#endif