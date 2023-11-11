#ifndef IMU_H
#define IMU_H

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <SoftwareSerial.h>
#include <Wire.h>

struct Vector {
    float x;
    float y;
    float z;
};

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