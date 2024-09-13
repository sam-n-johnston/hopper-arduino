#ifndef LEG_H
#define LEG_H

#include <Servo.h>
#include "puller.h"
#include "vector.h"
#include <SPI.h>
#include <Dynamixel2Arduino.h>

const float degToRad = 0.0174533;

class Leg {
private:
    Puller *puller;
    int servoXId;
    int servoYId;
    Dynamixel2Arduino dxl;
    int footSensorPin = 27;
    float servoXDesiredPositionDeg = 0.0;
    float servoYDesiredPositionDeg = 0.0;

public:
    Leg(Puller *puller,  int servoXId, int servoYId);

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