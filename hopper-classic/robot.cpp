#include "robot.h"

Robot::Robot(Leg *leg) { this->leg = leg; }

void Robot::begin()
{
    this->leg->begin();
    // this->leg->torqueOn();
    Serial.println("Robot Setup Done");
}

void Robot::updateStateIfChanged()
{
    if (this->currentState == STANCE_GOING_DOWN &&
        millis() - this->lastStateChangeTime > 5)
    {
        Serial.println("STANCE_GOING_UP...");
        this->currentState = STANCE_GOING_UP;
        this->lastStateChangeTime = millis();
        return;
    }
    if (this->currentState == STANCE_GOING_UP &&
        !this->leg->isFootTouchingGround())
    {
        Serial.println("FLIGHT_GOING_UP...");
        this->currentState = FLIGHT_GOING_UP;
        this->lastStateChangeTime = millis();
        return;
    }
    if (this->currentState == FLIGHT_GOING_UP &&
        millis() - this->lastStateChangeTime > 5)
    {
        Serial.println("FLIGHT_GOING_DOWN...");
        this->currentState = FLIGHT_GOING_DOWN;
        this->lastStateChangeTime = millis();
        return;
    }
    if (this->currentState == FLIGHT_GOING_DOWN &&
        this->leg->isFootTouchingGround())
    {
        Serial.println("STANCE_GOING_DOWN...");
        this->currentState = STANCE_GOING_DOWN;
        this->lastStateChangeTime = millis();
        return;
    }
}

bool Robot::hasFallen(float x, float y)
{
    return abs(x) > 40.0 || abs(y) > 40.0;
}

int Robot::getCurrentState() { return this->currentState; }

void Robot::sendCommandsToDuringStance(
    float bodyOrientationX, float bodyOrientationY)
{
    switch (this->currentState)
    {
    case STANCE_GOING_DOWN:
        this->moveLegToKeepRobotUpright(bodyOrientationX, bodyOrientationY);
        break;
    case STANCE_GOING_UP:
        this->leg->pushDown();
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
    float bodyOrientationYDot)
{
    switch (this->currentState)
    {
    case FLIGHT_GOING_UP:
        this->leg->stopPushingDown();
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

/**
 * @brief 
 * 
 * @param thetaX: X Angle in degree of the robot's body
 * @param thetaY: Y Angle in degree of the robot's body
 */
void Robot::moveLegToKeepRobotUpright(float thetaX, float thetaY)
{
    float alphaX = this->leg->getAlphaXInDeg();
    float alphaY = this->leg->getAlphaYInDeg();

    // Create PD control
    float kp = 0.05;

    /**
     * We want both thetas to be 0 degrees,
     * fix only a small percentage of the error
     * on each loop to prevent slippage of the foot
     */
    float desiredAlphaXRotation = kp * thetaX + alphaX;
    float desiredAlphaYRotation = kp * thetaY + alphaY;

    this->leg->setDesiredAlphaXYInDeg(
        desiredAlphaXRotation, desiredAlphaYRotation);
}

void Robot::moveLegForDesiredHorizontalSpeed(
    float xd,
    float yd,
    float bodyOrientationX,
    float bodyOrientationY,
    float thetaXDot,
    float thetaYDot)
{
    // Create PD control
    float k1 = 1.0;
    float k2 = 1.0;
    float k3 = 0.0;

    // To change based on commands from a controller
    float xdDesired = 0.0;
    float ydDesired = 0.0;
    float thetaDesired = 0.0;

    float xErr = k1 * (xd - xdDesired) +
                 k2 * (thetaDesired - bodyOrientationX) + k3 * (thetaXDot);
    float yErr = k1 * (xd - ydDesired) +
                 k2 * (thetaDesired - bodyOrientationY) + k3 * (thetaYDot);

    this->leg->setDesiredAlphaXYInDeg(xErr, yErr);
}

void Robot::stop() { leg->torqueOff(); }