#ifndef LEG_H
#define LEG_H

#include "puller.h"
#include "vector.h"
#include <SPI.h>

const float degToRad = 0.0174533;

class Leg {
private:
    Puller *puller;
    int footSensorPin = 27;

public:
    Leg(Puller *puller);

    void begin();
    void pushDown();
    void stopPushingDown();
    bool isFootTouchingGround();
    void torqueOff();
    void torqueOn();
};

#endif