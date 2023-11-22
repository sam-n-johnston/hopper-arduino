#include "leg.h"
#include "InverseKinematics.h"

Leg::Leg(IServo *servo1, IServo *servo2, IServo *servo3) {
    this->servo1 = servo1;
    this->servo2 = servo2;
    this->servo3 = servo3;
}

void Leg::begin() {
    this->servo1->begin();
    this->servo2->begin();
    this->servo3->begin();
}

void Leg::setFootPosition(Vector position) {
    setFootPosition(position.x, position.y, position.z);
}

void Leg::setFootPosition(float x, float y, float z) {
    float goalX = x;
    float goalY = y;
    float goalZ = z;

    float tempGoalFootExtension =
        sqrt(goalX * goalX + goalY * goalY + goalZ * goalZ);

    // check if it's a NaN
    if (tempGoalFootExtension == tempGoalFootExtension)
        goalFootExtension = tempGoalFootExtension;

    float theta1;
    float theta2;
    float theta3;

    int status = delta_calcInverse(x, y, z, theta1, theta2, theta3);

    // Check if angles are NaN
    if (status != 0 || theta1 != theta1 || theta2 != theta2 || theta3 != theta3)
        Serial.println("Failed to calculate inverse kinematics");
    else {
        this->servo1->setPositionInDeg(-theta1);
        this->servo2->setPositionInDeg(-theta2);
        this->servo3->setPositionInDeg(-theta3);
    }
}

Vector Leg::getFootPosition() {
    float x, y, z;

    int status = delta_calcForward(
        -servo1->getCurrentPosition(),
        -servo2->getCurrentPosition(),
        -servo3->getCurrentPosition(),
        x,
        y,
        z);

    if (status != 0)
        Serial.println("Failed to calculate forward kinematics");

    Vector position;

    position.x = x;
    position.y = y;
    position.z = z;

    footExtension = sqrt(x * x + y * y + z * z);

    return position;
}

float Leg::getAlphaXInDeg() {
    Vector pos = this->getFootPosition();

    return atan(-pos.y / pos.z);
}

float Leg::getAlphaYInDeg() {
    Vector pos = this->getFootPosition();

    return atan(pos.x / pos.z);
}

void Leg::setDesiredAlphaXYInDeg(float degX, float degY) {
    float newY = sin(degX * degToRad) * goalFootExtension;

    float projectedGoalFootExtensionX =
        sqrt(goalFootExtension * goalFootExtension - newY * newY);

    float newX = -sin(degY * degToRad) * projectedGoalFootExtensionX;
    float newZ = -sqrt(
        goalFootExtension * goalFootExtension - newY * newY - newX * newX);

    setFootPosition(newX, newY, newZ);
}

void Leg::setDesiredAlphaXYInDeg(
    float degX, float degY, float alphaXDeg, float alphaYDeg) {
    float newY = sin(degX * degToRad) * goalFootExtension -
                 sin(alphaXDeg * degToRad) * footLengthInMM;

    float projectedGoalFootExtensionX =
        sqrt(goalFootExtension * goalFootExtension - newY * newY);

    float newX = -sin(degY * degToRad) * projectedGoalFootExtensionX +
                 sin(alphaYDeg * degToRad) * footLengthInMM;
    float newZ = -sqrt(
        goalFootExtension * goalFootExtension - newY * newY - newX * newX);

    setFootPosition(newX, newY, newZ);
}

bool Leg::isFootTouchingGround() {
    int sensorValue = analogRead(this->footSensorPin);

    // Serial.print("Sensor value: ");
    // Serial.println(sensorValue);

    if (sensorValue > 100)
        return true;

    return false;
}

void Leg::torqueOff() {
    this->servo1->torqueOff();
    this->servo2->torqueOff();
    this->servo3->torqueOff();
}

void Leg::torqueOn() {
    this->servo1->torqueOn();
    this->servo2->torqueOn();
    this->servo3->torqueOn();
}
