#include "leg.h"

Leg::Leg(Puller *puller, Servo *servoX, Servo *servoY) {
    this->puller = puller;
    this->servoX = servoX;
    this->servoY = servoY;
}

void Leg::begin() {
    this->puller->begin();
    this->servoX->attach(18, 400, 2600);
    this->servoY->attach(19, 400, 2600);
}

/**
 * @brief 
 * 
 * @return float: returns the angle of the leg in relation to the body
 */
float Leg::getAlphaXInDeg() {
    return servoXDesiredPositionDeg;
}

/**
 * @brief 
 * 
 * @return float: returns the angle of the leg in relation to the body
 */
float Leg::getAlphaYInDeg() {
    return servoYDesiredPositionDeg;
}

void Leg::setDesiredAlphaXYInDeg(float degX, float degY) {
    float limit = 30.0;
    float xOffset = 1.5;
    float yOffset = -2.0;
    float desiredDegX = degX + xOffset;
    float desiredDegY = degY + yOffset;

    if (desiredDegX > limit)
        desiredDegX = limit;
    if (desiredDegX < -limit)
        desiredDegX = -limit;
    if (desiredDegY > limit)
        desiredDegY = limit;
    if (desiredDegY < -limit)
        desiredDegY = -limit;

    float limitedPositionY = 90.0 + desiredDegY;

    servoXDesiredPositionDeg = desiredDegX;
    servoYDesiredPositionDeg = desiredDegY;

    // Serial.print("Y: ");
    // Serial.println(desiredDegY);

    this->servoX->write(90.0 - desiredDegX);
    this->servoY->write(90.0 + desiredDegY);
}

void Leg::pushDown() {
    this->puller->setPositionInDeg(-85.0);
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
    this->servoX->detach();
    this->servoY->detach();
}

void Leg::torqueOn() {
    this->puller->torqueOn();
    this->servoX->attach(18, 400, 2600);
    this->servoY->attach(19, 400, 2600);
}
