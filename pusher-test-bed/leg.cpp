#include "leg.h"

Leg::Leg(Puller *puller) {
    this->puller = puller;
}

void Leg::begin() {
    this->puller->begin();
}

void Leg::pushDown() {
    this->puller->setPositionInDeg(90.0);
}

void Leg::stopPushingDown() {
    this->puller->setPositionInDeg(0.0);
}

bool Leg::isFootTouchingGround() {
    int sensorValue = analogRead(this->footSensorPin);

    // Serial.print("Sensor value: ");
    // Serial.println(sensorValue);

    if (sensorValue > 50)
        return true;

    return false;
}

void Leg::torqueOff() {
    this->puller->torqueOff();
}

void Leg::torqueOn() {
    this->puller->torqueOn();
}
