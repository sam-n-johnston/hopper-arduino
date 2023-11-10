#ifndef IMU_H
#define IMU_H

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <SoftwareSerial.h>

extern SoftwareSerial DEBUG_SERIAL;

struct Vector
{
    float x;
    float y;
    float z;
};

class IMU
{
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