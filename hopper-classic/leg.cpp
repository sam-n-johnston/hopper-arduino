#include "leg.h"

Leg::Leg(Puller *puller, Servo *servoX, Servo *servoY) {
    this->puller = puller;
    this->servoX = servoX;
    this->servoY = servoY;
}

void Leg::begin() {
    this->puller->begin();
    // this->servoX->attach(19, 400, 2600);
    // this->servoY->attach(18, 400, 2600);
}

// float Leg::getAlphaXInDeg() {
//     Vector pos = this->getFootPosition();

//     return atan(-pos.y / pos.z);
// }

// float Leg::getAlphaYInDeg() {
//     Vector pos = this->getFootPosition();

//     return atan(pos.x / pos.z);
// }

void Leg::setDesiredAlphaXYInDeg(float degX, float degY) {
    int limit = 30;
    int limitedPositionX = 90 + degX;
    int limitedPositionY = 90 + degY;
    if (limitedPositionX > 90 + limit)
        limitedPositionX = 90 + limit;
    if (limitedPositionX < 90 - limit)
        limitedPositionX = 90 - limit;
    if (limitedPositionY > 90 + limit)
        limitedPositionY = 90 + limit;
    if (limitedPositionY < 90 - limit)
        limitedPositionY = 90 - limit;

    this->servoX->write(limitedPositionX);
    this->servoY->write(limitedPositionY);
}

bool Leg::isFootTouchingGround() {
    int sensorValue = analogRead(this->footSensorPin);

    Serial.print("Sensor value: ");
    Serial.println(sensorValue);

    if (sensorValue > 50)
        return true;

    return false;
}

void Leg::torqueOff() {
    this->puller->torqueOff();
    this->servoX->detach();
    this->servoY->detach();
}

void Leg::torqueOn() {
    this->puller->torqueOn();
    this->servoX->attach(19, 400, 2600);
    this->servoY->attach(18, 400, 2600);
}
