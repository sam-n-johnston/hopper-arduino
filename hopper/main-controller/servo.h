#ifndef SERVO_H
#define SERVO_H

#include "AS5600.h"
#include "Wire.h"
#include <stdint.h>

class IServo {
public:
    virtual void begin(){};
    virtual void setPositionInDeg(float deg){};
    virtual void torqueOn(){};
    virtual void torqueOff(){};
    virtual int getCurrentPosition(){};
};

#endif
