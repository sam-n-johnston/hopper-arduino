#include "InverseKinematics.h"
#include "leg.h"
#include "localServo.h"
#include "spiServo.h"

const int SER1_PWM1 = 3, SER1_PWM2 = 5, SER1_OCM = A0, SER1_DIAG = 7,
          SER1_EN = 8, SER1_AS5600_MULTIPLEXER_PIN = -1;
const int SER2_AND_3_CHIP_SELECT_PIN = 10;

LocalServo servo1 = LocalServo(
    SER1_PWM1,
    SER1_PWM2,
    SER1_OCM,
    SER1_DIAG,
    SER1_EN,
    SER1_AS5600_MULTIPLEXER_PIN,
    1360,
    true);
SPIServo servo2 = SPIServo(
    SER2_AND_3_CHIP_SELECT_PIN,
    QUERY_GET_POSITION2,
    COMMAND_SET_GOAL_POSITION2,
    TORQUE_OFF2,
    TORQUE_ON2);
SPIServo servo3 = SPIServo(
    SER2_AND_3_CHIP_SELECT_PIN,
    QUERY_GET_POSITION3,
    COMMAND_SET_GOAL_POSITION3,
    TORQUE_OFF3,
    TORQUE_ON3);

Leg leg = Leg(&servo1, &servo2, &servo3);

void setup() {
    Serial.begin(115200);
    Serial.println("Setup Done");
    leg.begin();
    leg.torqueOn();
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

    leg.setPosition(0, 0, zValue);
}
