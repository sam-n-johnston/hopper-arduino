#ifndef IMU_H
#define IMU_H

#include "vector.h"
#include <Adafruit_BNO08x.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

class IMU {
private:
    Adafruit_BNO08x bno08x;
    unsigned long lastAccelerationMeasurementTimeInMs = 0;
    Vector lastComputedSpeed;
    Vector lastLinearAcceleration;
    Vector lastGravity;
    Vector lastOrientation;
    Vector lastAngularVelocity;
    sh2_SensorValue_t sensorValue;
    unsigned long currentMillis;
    unsigned long timeSinceLastMeasurementInMs;
    void setReports();
    Vector quaternionToEuler(float qr, float qi, float qj, float qk, bool degrees = false);

public:
    IMU();

    void begin();

    void getSensorData();
    Vector getLinearAcceleration();
    Vector getGravity();
    Vector getOrientation();
    Vector getComputedLinearVelocity();
    Vector getAngularVelocity();
};

#endif