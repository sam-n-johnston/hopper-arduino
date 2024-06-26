#include "imu.h"

IMU::IMU() {}

void IMU::begin()
{
    Serial.println("Starting IMU... ");
    Wire.begin();
    while (!bno08x.begin_I2C(BNO08x_I2CADDR_DEFAULT, &Wire))
    {
        Serial.println("Failed to find BNO08x chip");
        delay(100);
        // while (1)
        // {
        //     delay(10);
        // }
    }

    setReports();

    Serial.println("Done");
}

void IMU::setReports()
{

    // if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION))
    // {
    //     Serial.println("Could not enable linear acceleration");
    // }
    // if (!bno08x.enableReport(SH2_GRAVITY))
    // {
    //     Serial.println("Could not enable gravity vector");
    // }
    if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR))
    {
        Serial.println("Could not enable rotation vector");
    }
    // if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED))
    // {
    //     Serial.println("Could not enable gyroscope");
    // }
}

void IMU::getSensorData()
{

    if (bno08x.wasReset())
    {
        Serial.print("sensor was reset!");
        setReports();
    }

    if (!bno08x.getSensorEvent(&sensorValue))
    {
        // Serial.println("No IMU info returned!");
        return;
    }

    switch (sensorValue.sensorId)
    {
    // case SH2_LINEAR_ACCELERATION:
    //     lastLinearAcceleration.x = sensorValue.un.linearAcceleration.x;
    //     lastLinearAcceleration.y = sensorValue.un.linearAcceleration.y;
    //     lastLinearAcceleration.z = sensorValue.un.linearAcceleration.z;

    //     currentMillis = millis();

    //     timeSinceLastMeasurementInMs =
    //         currentMillis - this->lastAccelerationMeasurementTimeInMs;

    //     this->lastComputedSpeed.x =
    //         lastLinearAcceleration.x * timeSinceLastMeasurementInMs;
    //     this->lastComputedSpeed.y =
    //         lastLinearAcceleration.y * timeSinceLastMeasurementInMs;
    //     this->lastComputedSpeed.z =
    //         lastLinearAcceleration.z * timeSinceLastMeasurementInMs;

    //     this->lastAccelerationMeasurementTimeInMs = currentMillis;

    //     break;
    // case SH2_GRAVITY:
    //     lastGravity.x = sensorValue.un.gravity.x;
    //     lastGravity.y = -sensorValue.un.gravity.y;
    //     lastGravity.z = -sensorValue.un.gravity.z;
    //     break;
    case SH2_GAME_ROTATION_VECTOR:
        lastOrientation = quaternionToEuler(
            sensorValue.un.gameRotationVector.real,
            sensorValue.un.gameRotationVector.i,
            sensorValue.un.gameRotationVector.j,
            sensorValue.un.gameRotationVector.k,
            true);
        break;
    // case SH2_GYROSCOPE_CALIBRATED:
    //     lastAngularVelocity.x = -sensorValue.un.gyroscope.x;
    //     lastAngularVelocity.y = sensorValue.un.gyroscope.y;
    //     lastAngularVelocity.z = sensorValue.un.gyroscope.z;
    //     break;
    }
}

Vector IMU::quaternionToEuler(float qr, float qi, float qj, float qk, bool degrees)
{
    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    Vector tempVector;

    tempVector.z = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    tempVector.y = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    tempVector.x = -atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees)
    {
        tempVector.x *= RAD_TO_DEG;
        tempVector.y *= RAD_TO_DEG;
        tempVector.z *= RAD_TO_DEG;
    }

    return tempVector;
}

Vector IMU::getLinearAcceleration()
{
    // Serial.print("Got linear accelerations - x: ");
    // Serial.print(lastLinearAcceleration.x);
    // Serial.print(",\ty: ");
    // Serial.print(lastLinearAcceleration.y);
    // Serial.print(",\tz: ");
    // Serial.print(lastLinearAcceleration.z);
    // Serial.println();

    return lastLinearAcceleration;
}

Vector IMU::getGravity()
{
    // Serial.print("Got lastGravity - x: ");
    // Serial.print(lastGravity.x);
    // Serial.print(", y: ");
    // Serial.print(lastGravity.y);
    // Serial.print(", z: ");
    // Serial.print(lastGravity.z);
    // Serial.println();

    return lastGravity;
}

Vector IMU::getOrientation()
{
    // Serial.print("Got lastOrientation - x: ");
    // Serial.print(lastOrientation.x);
    // Serial.print(", y: ");
    // Serial.print(lastOrientation.y);
    // Serial.print(", z: ");
    // Serial.print(lastOrientation.z);
    // Serial.println();

    return lastOrientation;
}

Vector IMU::getComputedLinearVelocity() { return this->lastComputedSpeed; }

Vector IMU::getAngularVelocity()
{
    // Serial.print("Got lastAngularVelocity - x: ");
    // Serial.print(lastAngularVelocity.x);
    // Serial.print(",\ty: ");
    // Serial.print(lastAngularVelocity.y);
    // Serial.print(",\tz: ");
    // Serial.print(lastAngularVelocity.z);
    // Serial.println();

    return lastAngularVelocity;
}