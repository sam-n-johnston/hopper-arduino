#ifndef LEG_H
#define LEG_H

#include <Servo.h>
#include "puller.h"
#include "vector.h"
#include <SPI.h>

const float degToRad = 0.0174533;

class Leg {
private:
    Puller *puller;
    Servo *servoX;
    Servo *servoY;
    int footSensorPin = 27;
    float servoXDesiredPositionDeg = 0.0;
    float servoYDesiredPositionDeg = 0.0;

public:
    Leg(Puller *puller,  Servo *servoX, Servo *servoY);

    void begin();
    float getAlphaXInDeg();
    float getAlphaYInDeg();
    void setDesiredAlphaXYInDeg(float degX, float degY);
    void pushDown();
    void stopPushingDown();
    bool isFootTouchingGround();
    void torqueOff();
    void torqueOn();
};

#endif