#ifndef IK_H
#define IK_H

#include "math.h"

int delta_calcForward(
    float theta1, float theta2, float theta3, float &x0, float &y0, float &z0);
int delta_calcInverse(
    float x0, float y0, float z0, float &theta1, float &theta2, float &theta3);

#endif
