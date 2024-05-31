#include "leg.h"

Leg::Leg(Puller *puller) {
    this->puller = puller;
}

void Leg::begin() {
    this->puller->begin();
    this->servoX->attach(18, 400, 2600);
    this->servoY->attach(19, 400, 2600);
}

// void Leg::setFootPosition(Vector position) {
//     setFootPosition(position.x, position.y, position.z);
// }

// void Leg::setPushFactor(float pushFactor) { this->pushFactor = pushFactor; }

// void Leg::setFootPosition(float x, float y, float z) {
//     // NOOP
// }

// Vector Leg::getFootPosition() {

//     return position;
// }

// float Leg::getAlphaXInDeg() {
//     Vector pos = this->getFootPosition();

//     return atan(-pos.y / pos.z);
// }

// float Leg::getAlphaYInDeg() {
//     Vector pos = this->getFootPosition();

//     return atan(pos.x / pos.z);
// }

void Leg::setDesiredAlphaXYInDeg(float degX, float degY) {
    this->servoX->write(degX);
    this->servoY->write(degY);
}

// void Leg::setDesiredAlphaXYInDeg(
//     float degX, float degY, float alphaXDeg, float alphaYDeg) {
//     // float desiredGoal = goalFootExtension * this->pushFactor;

//     float newY = sin(degX * degToRad) * goalFootExtension -
//                  sin(alphaXDeg * degToRad) * footLengthInMM;

//     float projectedGoalFootExtensionX =
//         sqrt(goalFootExtension * goalFootExtension - newY * newY);

//     float newX = -sin(degY * degToRad) * projectedGoalFootExtensionX +
//                  sin(alphaYDeg * degToRad) * footLengthInMM;
//     float newZ = -sqrt(
//         goalFootExtension * goalFootExtension - newY * newY - newX * newX);

//     // BAD = These values aren't degrees.
//     // Serial.print("X: ");
//     // Serial.print(degX);
//     // Serial.print("\tNewY: ");
//     // Serial.print(newY);
//     // Serial.print("\tNewX: ");
//     // Serial.print(newX);
//     // Serial.print("\tNewZ: ");
//     // Serial.println(newZ);

//     setFootPosition(newX, newY, newZ);
// }

bool Leg::isFootTouchingGround() {
    int sensorValue = analogRead(this->footSensorPin);

    // Serial.print("Sensor value: ");
    // Serial.println(sensorValue);

    if (sensorValue > 50)
        return true;

    return false;
}

void Leg::torqueOff() {
    // this->servo1->torqueOff();
    // this->servo2->torqueOff();
    // this->servo3->torqueOff();
}

void Leg::torqueOn() {
    // this->servo1->torqueOn();
    // this->servo2->torqueOn();
    // this->servo3->torqueOn();
}
