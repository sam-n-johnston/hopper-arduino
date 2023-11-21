#ifndef LOCAL_SERVO_H
#define LOCAL_SERVO_H

#include "AS5600.h"
#include "Wire.h"
#include "servo.h"
#include <stdint.h>

class LocalServo : public IServo {
private:
    uint8_t PWM1;
    uint8_t PWM2;
    uint8_t OCM;
    uint8_t DIAG;
    uint8_t EN;
    int zeroPosition = 0;
    bool direction;
    int previousPosition = 0;
    unsigned long previousPositionTime = 0;
    int deltaTime = 0;
    float integralError = 0.0;
    float previousError = 0.0;
    int currentTurn = 0;
    AS5600 as5600;

    void setMotorTorque(float speed);
    float getPIDOutput(float error);

public:
    LocalServo(
        uint8_t PWM1,
        uint8_t PWM2,
        uint8_t OCM,
        uint8_t DIAG,
        uint8_t EN,
        int zeroPosition,
        bool direction);

    void begin();
    void setPositionInDeg(float deg);
    void torqueOn();
    void torqueOff();
    float getCurrentPosition();
};

#endif