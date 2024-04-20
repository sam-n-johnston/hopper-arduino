#include "leg.h"
#include "InverseKinematics.h"

Leg::Leg(Servo *servo1, Servo *servo2, Servo *servo3) {
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

void Leg::setPushFactor(float pushFactor) { this->pushFactor = pushFactor; }

void Leg::setFootPosition(float x, float y, float z) {
    float goalX = x;
    float goalY = y;
    float goalZ = z;

    float tempGoalFootExtension =
        sqrt(goalX * goalX + goalY * goalY + goalZ * goalZ);

    // check if it's a NaN
    if (tempGoalFootExtension == tempGoalFootExtension) {
        goalFootExtension = tempGoalFootExtension;
        // Serial.print("Foot extension - x: ");
        // Serial.print(goalX);
        // Serial.print("\ty:");
        // Serial.print(y);
        // Serial.print("\tz:");
        // Serial.println(z);
        // Serial.print("tempGoalFootExtension is NaN");
    }

    float theta1;
    float theta2;
    float theta3;

    int status = delta_calcInverse(
        x * pushFactor, y * pushFactor, z * pushFactor, theta1, theta2, theta3);

    // Check if angles are NaN
    if (status != 0 || theta1 != theta1 || theta2 != theta2 || theta3 != theta3) {
        Serial.print("Failed to calculate inverse kinematics - x: ");
        Serial.print(x * pushFactor);
        Serial.print(" - y: ");
        Serial.print(y * pushFactor);
        Serial.print(" - z: ");
        Serial.print(z * pushFactor);
        Serial.println(";");

        this->servo1->setPositionInDeg(0);
        this->servo2->setPositionInDeg(0);
        this->servo3->setPositionInDeg(0);
    } else {
        // Serial.print("Setting positions - 1: ");
        // Serial.print(-theta1);
        // Serial.print("\t2:");
        // Serial.print(-theta2);
        // Serial.print("\t3:");
        // Serial.println(-theta3);
        this->servo1->setPositionInDeg(-theta1);
        this->servo2->setPositionInDeg(-theta2);
        this->servo3->setPositionInDeg(-theta3);
    }

}

Vector Leg::getFootPosition() {
    float x, y, z;

    int status = delta_calcForward(
        -servo1->getMostRecentPosition(),
        -servo2->getMostRecentPosition(),
        -servo3->getMostRecentPosition(),
        x,
        y,
        z);

    if (status != 0) {
        Serial.print("Failed to calculate forward kinematics: ");
        Serial.print(servo1->getMostRecentPosition());
        Serial.print(" - ");
        Serial.print(servo2->getMostRecentPosition());
        Serial.print(" - ");
        Serial.println(servo3->getMostRecentPosition());
    }

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

void Leg::setDesiredAlphaXYInDeg(float degX, float degY) {
    // float desiredGoal = goalFootExtension * this->pushFactor;

    float newY = sin(degX * degToRad) * goalFootExtension;

    float projectedGoalFootExtensionX =
        sqrt(goalFootExtension * goalFootExtension - newY * newY);

    float newX = -sin(degY * degToRad) * projectedGoalFootExtensionX;
    float newZ = -sqrt(
        goalFootExtension * goalFootExtension - newY * newY - newX * newX);

    // Serial.print("x");
    // Serial.print(newX);
    // Serial.print("y");
    // Serial.print(newY);
    // Serial.print("z");
    // Serial.print(newZ);
    // Serial.println("");

    setFootPosition(newX, newY, newZ);
}

void Leg::setDesiredAlphaXYInDeg(
    float degX, float degY, float alphaXDeg, float alphaYDeg) {
    // float desiredGoal = goalFootExtension * this->pushFactor;

    float newY = sin(degX * degToRad) * goalFootExtension -
                 sin(alphaXDeg * degToRad) * footLengthInMM;

    float projectedGoalFootExtensionX =
        sqrt(goalFootExtension * goalFootExtension - newY * newY);

    float newX = -sin(degY * degToRad) * projectedGoalFootExtensionX +
                 sin(alphaYDeg * degToRad) * footLengthInMM;
    float newZ = -sqrt(
        goalFootExtension * goalFootExtension - newY * newY - newX * newX);

    // BAD = These values aren't degrees.
    // Serial.print("X: ");
    // Serial.print(degX);
    // Serial.print("\tNewY: ");
    // Serial.print(newY);
    // Serial.print("\tNewX: ");
    // Serial.print(newX);
    // Serial.print("\tNewZ: ");
    // Serial.println(newZ);

    setFootPosition(newX, newY, newZ);
}

bool Leg::isFootTouchingGround() {
    int sensorValue = analogRead(this->footSensorPin);

    // Serial.print("Sensor value: ");
    // Serial.println(sensorValue);

    if (sensorValue < 1000)
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
