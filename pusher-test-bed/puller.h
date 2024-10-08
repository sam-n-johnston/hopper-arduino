#ifndef LOCAL_PULLER_H
#define LOCAL_PULLER_H

#include "AS5600.h"
#include "Wire.h"
#include <stdint.h>

class Puller {
private:
    uint8_t PWM1;
    uint8_t PWM2;
    uint8_t OCM;
    uint8_t DIAG;
    uint8_t EN;
    int zeroPosition = 0;
    bool direction;
    int previousPosition = 0;
    int positionDelta = 0;
    unsigned long previousPositionTime = 0;
    int deltaTime = 0;
    float desiredPosition = 0.0;
    float integralError = 0.0;
    float previousError = 0.0;
    float mostRecentPosition = 0.0;
    int currentTurn = 0;
    AS5600 as5600;
    float getPIDOutput(float error);
    void findZeroPosition();
    float getMotorCurrent();

public:
    Puller(
        uint8_t PWM1,
        uint8_t PWM2,
        uint8_t OCM,
        uint8_t DIAG,
        uint8_t EN,
        bool direction,
        int zeroPosition);

    float getCurrentPosition();
    void setMotorTorque(float speed);
    void begin();
    void setPositionInDeg(float deg);
    void goToDesiredPosition();
    void torqueOn();
    void torqueOff();
    float getMostRecentPosition();
};

#endif