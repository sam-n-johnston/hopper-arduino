#include "imu.h"

IMU::IMU() {}

void IMU::begin() {
    Serial.print("Starting IMU... ");
    this->bno = Adafruit_BNO055(55, 0x28, &Wire);

    if (!this->bno.begin()) {
        Serial.print(
            "Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1)
            ;
    }

    // Wire.setClock(400000);
    Serial.println("Setup done");
}

Vector IMU::getLinearAcceleration() {
    sensors_event_t linearAccelData, gravityData;
    this->bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

    Vector linearAcceleration;
    linearAcceleration.x = linearAccelData.acceleration.x;
    linearAcceleration.y = linearAccelData.acceleration.y;
    linearAcceleration.z = linearAccelData.acceleration.z;
    unsigned long timeSinceLastMeasurementInMs =
        millis() - this->lastAccelerationMeasurementTimeInMs;

    this->lastComputedSpeed.x =
        linearAcceleration.x * timeSinceLastMeasurementInMs;
    this->lastComputedSpeed.y =
        linearAcceleration.y * timeSinceLastMeasurementInMs;
    this->lastComputedSpeed.z =
        linearAcceleration.z * timeSinceLastMeasurementInMs;

    this->lastAccelerationMeasurementTimeInMs = millis();

    // Serial.print("Got linear accelerations - x: ");
    // Serial.print(linearAcceleration.x);
    // Serial.print(", y: ");
    // Serial.print(linearAcceleration.y);
    // Serial.print(", z: ");
    // Serial.print(linearAcceleration.z);
    // Serial.println();

    return linearAcceleration;
}

Vector IMU::getGravity() {
    sensors_event_t gravityData;
    bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

    Vector gravity;
    gravity.x = -gravityData.acceleration.x;
    gravity.y = -gravityData.acceleration.y;
    gravity.z = -gravityData.acceleration.z;

    // Serial.print("Got gravity - x: ");
    // Serial.print(gravity.x);
    // Serial.print(", y: ");
    // Serial.print(gravity.y);
    // Serial.print(", z: ");
    // Serial.print(gravity.z);
    // Serial.println();

    return gravity;
}

Vector IMU::getOrientation() {
    sensors_event_t orientationData;
    this->bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

    Vector orientation;
    orientation.x = -orientationData.orientation.y;
    orientation.y = orientationData.orientation.z;
    orientation.z = -orientationData.orientation.x;

    // Serial.print("Got orientation - x: ");
    // Serial.print(orientation.x);
    // Serial.print(", y: ");
    // Serial.print(orientation.y);
    // Serial.print(", z: ");
    // Serial.print(orientation.z);
    // Serial.println();

    return orientation;
}

Vector IMU::getComputedLinearVelocity() { return this->lastComputedSpeed; }

Vector IMU::getAngularVelocity() {
    sensors_event_t angVelocityData;
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);

    Vector angularVelocity;
    angularVelocity.x = -angVelocityData.gyro.z;
    angularVelocity.y = angVelocityData.gyro.y;
    angularVelocity.z = -angVelocityData.gyro.x;

    // Serial.print("Got angularVelocity - x: ");
    // Serial.print(angularVelocity.x);
    // Serial.print(", y: ");
    // Serial.print(angularVelocity.y);
    // Serial.print(", z: ");
    // Serial.print(angularVelocity.z);
    // Serial.println();

    return angularVelocity;
}