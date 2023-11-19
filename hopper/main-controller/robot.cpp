#include "robot.h"

Robot::Robot(Leg *leg) { this->leg = leg; }

void Robot::begin() {
    Serial.println("Starting robot...");

    this->leg->begin();
    this->leg->torqueOn();
    Serial.println("Robot Started.");
}

void Robot::updateStateIfChanged() {
    if (this->currentState == STANCE_GOING_DOWN &&
        millis() - this->lastStateChangeTime > 10) {
        Serial.println("STANCE_GOING_UP...");
        this->currentState = STANCE_GOING_UP;
        this->lastStateChangeTime = millis();
        return;
    }
    if (this->currentState == STANCE_GOING_UP &&
        !this->leg->isFootTouchingGround()) {
        Serial.println("FLIGHT_GOING_UP...");
        this->currentState = FLIGHT_GOING_UP;
        this->lastStateChangeTime = millis();
        return;
    }
    if (this->currentState == FLIGHT_GOING_UP &&
        millis() - this->lastStateChangeTime > 200) {
        Serial.println("FLIGHT_GOING_DOWN...");
        this->currentState = FLIGHT_GOING_DOWN;
        this->lastStateChangeTime = millis();
        return;
    }
    if (this->currentState == FLIGHT_GOING_DOWN &&
        this->leg->isFootTouchingGround()) {
        Serial.println("STANCE_GOING_DOWN...");
        this->currentState = STANCE_GOING_DOWN;
        this->lastStateChangeTime = millis();
        return;
    }
}

bool Robot::hasFallen(Vector gravity) { return gravity.z > -6.0; }

int Robot::getCurrentState() { return this->currentState; }

void Robot::sendCommandsToDuringStance(
    float bodyOrientationX, float bodyOrientationY) {
    switch (this->currentState) {
    case STANCE_GOING_DOWN:
        // leg->movePusherToPosition(0.0);
        this->moveLegToKeepRobotUpright(bodyOrientationX, bodyOrientationY);
        break;
    case STANCE_GOING_UP:
        // TODO: push hard
        // leg->movePusherToPosition(30.0);
        this->moveLegToKeepRobotUpright(bodyOrientationX, bodyOrientationY);
        break;

    default:
        break;
    }
}

void Robot::sendCommandsToMotorsDuringFlight(
    float xd,
    float yd,
    float bodyOrientationX,
    float bodyOrientationY,
    float bodyOrientationXDot,
    float bodyOrientationYDot) {
    switch (this->currentState) {
    case FLIGHT_GOING_UP:
        this->moveLegForDesiredHorizontalSpeed(
            xd,
            yd,
            bodyOrientationX,
            bodyOrientationY,
            bodyOrientationXDot,
            bodyOrientationYDot);
        break;
    case FLIGHT_GOING_DOWN:
        this->moveLegForDesiredHorizontalSpeed(
            xd,
            yd,
            bodyOrientationX,
            bodyOrientationY,
            bodyOrientationXDot,
            bodyOrientationYDot);
        break;

    default:
        break;
    }
}

void Robot::moveLegToKeepRobotUpright(float thetaX, float thetaY) {
    float alphaX = this->leg->getAlphaXInDeg();
    float alphaY = this->leg->getAlphaYInDeg();

    float alphaDesiredX = alphaX + thetaX;
    float alphaDesiredY = alphaY + thetaY;

    // Create PD control
    float kp = 0.5;

    /**
     * We want both thetas to be 0 degrees,
     * fix only a small percentage of the error
     * on each loop to prevent slippage of the foot
     */
    float desiredAlphaXRotation = kp * (alphaDesiredX - alphaX) + alphaX;
    float desiredAlphaYRotation = kp * (alphaDesiredY - alphaY) + alphaY;

    this->leg->setDesiredAlphaXYInDeg(
        desiredAlphaXRotation, desiredAlphaYRotation);
}

void Robot::moveLegForDesiredHorizontalSpeed(
    float xd,
    float yd,
    float bodyOrientationX,
    float bodyOrientationY,
    float thetaXDot,
    float thetaYDot) {
    // Create PD control
    float k1 = 1.0;
    float k2 = 1.0;
    float k3 = 0.0;

    // To change based on commands from a controller
    float xdDesired = 0.0;
    float ydDesired = 0.0;
    float thetaDesired = 0.0;

    float xErr = k1 * (xd - xdDesired) +
                 k2 * (bodyOrientationX - thetaDesired) + k3 * (thetaXDot);
    float yErr = k1 * (xd - ydDesired) +
                 k2 * (bodyOrientationY - thetaDesired) + k3 * (thetaYDot);

    // Compute kinematics to gets to x & y desired foot position
    float alphaXDesired = 0; // asin(xErr / this->footLengthInMM);
    float alphaYDesired = 0; // asin(yErr / this->footLengthInMM);

    this->leg->setDesiredAlphaXYInDeg(
        alphaXDesired - bodyOrientationX, alphaYDesired - bodyOrientationY);
}

void Robot::stop() { leg->torqueOff(); }