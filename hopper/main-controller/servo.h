#ifndef LOCAL_SERVO_H
#define LOCAL_SERVO_H

#include "AS5600.h"
#include "Wire.h"
#include <stdint.h>

class Servo {
private:
    uint8_t PWM1;
    uint8_t PWM2;
    uint8_t OCM;
    uint8_t DIAG;
    uint8_t EN;
    uint8_t as5600MultiplexerPin;
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
    float getCurrentPosition();

public:
    Servo(
        uint8_t PWM1,
        uint8_t PWM2,
        uint8_t OCM,
        uint8_t DIAG,
        uint8_t EN,
        uint8_t as5600MultiplexerPin,
        int zeroPosition,
        bool direction);

    void setMotorTorque(float speed);
    void begin();
    void setPositionInDeg(float deg);
    void goToDesiredPosition();
    void torqueOn();
    void torqueOff();
    float getMostRecentPosition();
};

#endif