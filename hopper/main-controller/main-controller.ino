#include "InverseKinematics.h"
#include "localServo.h"
#include "spiServo.h"

const int SER1_PWM1 = 3, SER1_PWM2 = 5, SER1_OCM = A0, SER1_DIAG = 7,
          SER1_EN = 8;
const int SER2_AND_3_CHIP_SELECT_PIN = 10;

Servo servo1 = Servo(
    SER1_PWM1,
    SER1_PWM2,
    SER1_OCM,
    SER1_DIAG,
    SER1_EN,
    SER1_AS5600,
    1360,
    true);
Servo servo2 = Servo(
    SER2_AND_3_CHIP_SELECT_PIN,
    QUERY_GET_POSITION2,
    COMMAND_SET_GOAL_POSITION2,
    TORQUE_OFF2,
    TORQUE_ON2);
Servo servo3 = Servo(
    SER2_AND_3_CHIP_SELECT_PIN,
    QUERY_GET_POSITION3,
    COMMAND_SET_GOAL_POSITION3,
    TORQUE_OFF3,
    TORQUE_ON3);

Leg leg = Leg(servo1, servo2, servo3);

void setup() {
    Serial.begin(115200);
    leg.begin();
}

int lastSecond = 0;
int loops = 0;

void loop() {
    loops++;
    long int currTime = millis();

    if (lastSecond + 1000 < currTime) {
        lastSecond = currTime;
        Serial.print("Current hz: ");
        Serial.println(loops);
        loops = 0;
    }

    long int time = millis();

    float theta1;
    float theta2;
    float theta3;

    float zValue = -100.0 - 25.0 * sin(millis() / 100.0);

    int status = delt a_calcInverse(
        0, 0, static_cast<int>(zValue), theta1, theta2, theta3);

    servo1.setPositionInDeg(-theta1);
    servo2.setPositionInDeg(-theta2);
    servo3.setPositionInDeg(-theta3);
}
