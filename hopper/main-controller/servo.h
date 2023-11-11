#ifndef SERVO_H
#define SERVO_H

#include "AS5600.h"
#include "Wire.h"
#include <stdint.h>

class IServo {
public:
    virtual void begin() = 0;
    virtual void setPositionInDeg(float deg) = 0;
    virtual void torqueOn() = 0;
    virtual void torqueOff() = 0;
    virtual int getCurrentPosition() = 0;
};

#endif
