#ifndef ROBOT_H
#define ROBOT_H

#include "leg.h"

const int STANCE_GOING_DOWN = 1;
const int STANCE_GOING_UP = 2;
const int FLIGHT_GOING_UP = 3;
const int FLIGHT_GOING_DOWN = 4;

class Robot {
private:
    const float footLengthInMM = 0.0;
    int totalExecutionCounts = 0;
    unsigned long totalServoErrorX = 0;
    unsigned long totalServoErrorY = 0;
    unsigned long timeOfLastServoErrorPrint = 0;

    Leg *leg;
    int currentState = FLIGHT_GOING_DOWN;
    bool isWithin(float val1, float val2, float interval);
    void printServoError(float desiredValueX, float desiredValueY);
    unsigned long lastStateChangeTime = 0;

public:
    Robot(Leg *leg);

    void begin();
    void stop();
    void updateStateIfChanged();
    bool hasFallen(float x, float y);
    int getCurrentState();
    void sendCommandsToDuringStance();
    void sendCommandsToMotorsDuringFlight();
};

#endif