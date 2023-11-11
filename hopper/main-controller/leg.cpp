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

    this->servo1->setPositionInDeg(-theta1);
    this->servo2->setPositionInDeg(-theta2);
    this->servo3->setPositionInDeg(-theta3);
}

bool Leg::isFootTouchingGround() {
    int sensorPin = A0;
    int sensorValue = analogRead(sensorPin);

    // Serial.print("Sensor value: ");
    // Serial.println(sensorValue);

    if (sensorValue > 350)
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
