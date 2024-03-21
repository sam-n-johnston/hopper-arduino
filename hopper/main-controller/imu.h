#ifndef IMU_H
#define IMU_H

#include "vector.h"
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

class IMU {
private:
    Adafruit_BNO055 bno;
    unsigned long lastAccelerationMeasurementTimeInMs = 0;
    Vector lastComputedSpeed;

public:
    IMU();

    void begin();

    Vector getLinearAcceleration();
    Vector getGravity();
    Vector getOrientation();
    Vector getComputedLinearVelocity();
    Vector getAngularVelocity();
};

#endif