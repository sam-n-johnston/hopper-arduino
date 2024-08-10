#include "robot.h"

Robot::Robot(Leg *leg) { this->leg = leg; }

void Robot::begin()
{
    this->leg->begin();
    this->leg->torqueOn();
    Serial.println("Robot Setup Done");
}

void Robot::updateStateIfChanged()
{
    if (this->currentState == STANCE_GOING_DOWN &&
        millis() - this->lastStateChangeTime > 1)
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

void Robot::sendCommandsToDuringStance()
{
    switch (this->currentState)
    {
    case STANCE_GOING_DOWN:
        break;
    case STANCE_GOING_UP:
        this->leg->pushDown();
        break;

    default:
        break;
    }
}

void Robot::sendCommandsToMotorsDuringFlight()
{
    switch (this->currentState)
    {
    case FLIGHT_GOING_UP:
        this->leg->stopPushingDown();
        break;
    case FLIGHT_GOING_DOWN:
        break;

    default:
        break;
    }
}

void Robot::stop() { leg->torqueOff(); }