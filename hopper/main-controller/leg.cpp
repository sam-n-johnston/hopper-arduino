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

void Leg::setPosition(float x, float y, float z) {
    float theta1;
    float theta2;
    float theta3;

    int status = delta_calcInverse(
        static_cast<int>(x),
        static_cast<int>(y),
        static_cast<int>(z),
        theta1,
        theta2,
        theta3);

    Serial.print("positions: ");
    Serial.println(-theta3);

    this->servo1->setPositionInDeg(-theta1);
    this->servo2->setPositionInDeg(-theta2);
    this->servo3->setPositionInDeg(-theta3);
}

Vector Leg::getFootPosition() {
    float x, y, z;

    int status = delta_calcForward(
        servo1->getCurrentPosition(),
        servo2->getCurrentPosition(),
        servo3->getCurrentPosition(),
        x,
        y,
        z);

    if (status != 0)
        Serial.println("Failed to calculate forward kinematics");

    Vector position;

    position.x = x;
    position.y = y;
    position.z = z;

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

void Leg::setDesiredAlphaXInDeg(float deg) {
    Vector pos = this->getFootPosition();

    float newY = -tan(deg) * pos.z;

    setPosition(pos.x, newY, pos.z);
}

void Leg::setDesiredAlphaYInDeg(float deg) {
    Vector pos = this->getFootPosition();

    float newX = tan(deg) * pos.z;

    setPosition(newX, pos.y, pos.z);
}

void Leg::pushByFactor(float factor) {
    Vector pos = this->getFootPosition();

    setPosition(factor * pos.x, factor * pos.y, factor * pos.z);
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
