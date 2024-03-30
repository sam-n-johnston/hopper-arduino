#include "imu.h"

IMU::IMU() {}

void IMU::begin()
{
    Serial.println("Starting IMU... ");
    Wire.setSDA(0);
    Wire.setSCL(1);
    if (!bno08x.begin_I2C(BNO08x_I2CADDR_DEFAULT, &Wire))
    {
        Serial.println("Failed to find BNO08x chip");
        while (1)
        {
            delay(10);
        }
    }

    setReports();

    Serial.println("Done");
}

void IMU::setReports()
{

    if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION))
    {
        Serial.println("Could not enable linear acceleration");
    }
    if (!bno08x.enableReport(SH2_GRAVITY))
    {
        Serial.println("Could not enable gravity vector");
    }
    if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR))
    {
        Serial.println("Could not enable rotation vector");
    }
    if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED))
    {
        Serial.println("Could not enable gyroscope");
    }
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
        return;
    }

    switch (sensorValue.sensorId)
    {
    case SH2_LINEAR_ACCELERATION:
        lastLinearAcceleration.x = sensorValue.un.linearAcceleration.x;
        lastLinearAcceleration.y = sensorValue.un.linearAcceleration.y;
        lastLinearAcceleration.z = sensorValue.un.linearAcceleration.z;

        currentMillis = millis();

        timeSinceLastMeasurementInMs =
            currentMillis - this->lastAccelerationMeasurementTimeInMs;

        this->lastComputedSpeed.x =
            lastLinearAcceleration.x * timeSinceLastMeasurementInMs;
        this->lastComputedSpeed.y =
            lastLinearAcceleration.y * timeSinceLastMeasurementInMs;
        this->lastComputedSpeed.z =
            lastLinearAcceleration.z * timeSinceLastMeasurementInMs;

        this->lastAccelerationMeasurementTimeInMs = currentMillis;

        break;
    case SH2_GRAVITY:
        lastGravity.x = sensorValue.un.gravity.x;
        lastGravity.y = -sensorValue.un.gravity.y;
        lastGravity.z = -sensorValue.un.gravity.z;
        break;
    case SH2_GAME_ROTATION_VECTOR:
        lastOrientation.x = -sensorValue.un.gameRotationVector.i;
        lastOrientation.y = sensorValue.un.gameRotationVector.j;
        lastOrientation.z = sensorValue.un.gameRotationVector.k;
        break;
    case SH2_GYROSCOPE_CALIBRATED:
        lastAngularVelocity.x = -sensorValue.un.gyroscope.x;
        lastAngularVelocity.y = sensorValue.un.gyroscope.y;
        lastAngularVelocity.z = sensorValue.un.gyroscope.z;
        break;
    }
}

Vector IMU::getLinearAcceleration()
{
    Serial.print("Got linear accelerations - x: ");
    Serial.print(lastLinearAcceleration.x);
    Serial.print(",\ty: ");
    Serial.print(lastLinearAcceleration.y);
    Serial.print(",\tz: ");
    Serial.print(lastLinearAcceleration.z);
    Serial.println();

    return lastLinearAcceleration;
}

Vector IMU::getGravity()
{
    Serial.print("Got lastGravity - x: ");
    Serial.print(lastGravity.x);
    Serial.print(", y: ");
    Serial.print(lastGravity.y);
    Serial.print(", z: ");
    Serial.print(lastGravity.z);
    Serial.println();

    return lastGravity;
}

Vector IMU::getOrientation()
{
    Serial.print("Got lastOrientation - x: ");
    Serial.print(lastOrientation.x);
    Serial.print(", y: ");
    Serial.print(lastOrientation.y);
    Serial.print(", z: ");
    Serial.print(lastOrientation.z);
    Serial.println();

    return lastOrientation;
}

Vector IMU::getComputedLinearVelocity() { return this->lastComputedSpeed; }

Vector IMU::getAngularVelocity()
{
    Serial.print("Got lastAngularVelocity - x: ");
    Serial.print(lastAngularVelocity.x);
    Serial.print(",\ty: ");
    Serial.print(lastAngularVelocity.y);
    Serial.print(",\tz: ");
    Serial.print(lastAngularVelocity.z);
    Serial.println();

    return lastAngularVelocity;
}