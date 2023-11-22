#include "InverseKinematics.h"
#include "imu.h"
#include "leg.h"
#include "localServo.h"
#include "robot.h"
#include "spiServo.h"

const int SER2_PWM1 = 3, SER2_PWM2 = 5, SER2_OCM = A0, SER2_DIAG = 7,
          SER2_EN = 8;
const int SER1_AND_3_CHIP_SELECT_PIN = 10;

IMU customImu = IMU();

SPIServo servo1 = SPIServo(
    SER1_AND_3_CHIP_SELECT_PIN,
    QUERY_GET_POSITION1,
    COMMAND_SET_GOAL_POSITION1,
    TORQUE_OFF1,
    TORQUE_ON1);
LocalServo servo2 =
    LocalServo(SER2_PWM1, SER2_PWM2, SER2_OCM, SER2_DIAG, SER2_EN, 1030, true);
SPIServo servo3 = SPIServo(
    SER1_AND_3_CHIP_SELECT_PIN,
    QUERY_GET_POSITION3,
    COMMAND_SET_GOAL_POSITION3,
    TORQUE_OFF3,
    TORQUE_ON3);

Leg leg = Leg(&servo1, &servo2, &servo3);
Robot robot = Robot(&leg);

void setup() {
    Serial.begin(115200);
    Serial.println("Setup Done");

    robot.begin();
    customImu.begin();

    Wire.setClock(1000000);

    leg.setFootPosition(0, 0, -100);

    Serial.println("Setup Done");
}

long lastSecond = 0;
long loops = 0;
long lastMillisIMU = 0;
bool robotFell = false;
Vector bodyOrientation;
Vector linearAcceleration;
Vector linearVelocity;
Vector angularVelocity;

void loop() {
    loops++;
    long currTime = millis();

    if (lastSecond + 1000 < currTime) {
        lastSecond = currTime;
        Serial.print("Current hz: ");
        Serial.println(loops);
        loops = 0;
    }

    // if (millis() < 3000)
    // leg.setFootPosition(0, 0, -100);
    // leg.getFootPosition();
    // else
    // float valX = 15.0 * sin(millis() / 100.0);
    // float valY = 15.0 * cos(millis() / 100.0);

    // leg.setDesiredAlphaXYInDeg(valX, valY);
    // leg.setDesiredAlphaYInDeg(0.0);

    // int pos = servo1.getCurrentPosition();

    // Serial.print("positions: ");
    // Serial.println(pos);

    if (!robotFell) {
        if (lastMillisIMU + 50 < currTime) {
            lastMillisIMU = currTime;
            bool isFootTouchingGround = leg.isFootTouchingGround();

            if (isFootTouchingGround) {
                bodyOrientation = customImu.getOrientation();
            } else {
                bodyOrientation = customImu.getOrientation();
                linearVelocity = customImu.getComputedLinearVelocity();
                angularVelocity = customImu.getAngularVelocity();
            }
        }

        robot.updateStateIfChanged();
        // if (robot.getCurrentState() == STANCE_GOING_DOWN ||
        //     robot.getCurrentState() == STANCE_GOING_UP) {
        //     robot.sendCommandsToDuringStance(
        //         bodyOrientation.x, bodyOrientation.y);
        // } else {

        //     robot.sendCommandsToMotorsDuringFlight(
        //         linearVelocity.x,
        //         linearVelocity.y,
        //         bodyOrientation.x,
        //         bodyOrientation.y,
        //         angularVelocity.x,
        //         angularVelocity.y);
        // }

        // Serial.print("Got orientation - x: ");
        // Serial.print(bodyOrientation.x);
        // Serial.print("; y: ");
        // Serial.print(bodyOrientation.y);
        // Serial.println();

        float val1 = -25.0 * sin(millis() / 100.0);
        // float val1 = -100.0 - 25.0 * sin(millis() / 1000.0);

        float theta1;
        float theta2;
        float theta3;

        int status = delta_calcInverse(val1, 0, -100, theta1, theta2, theta3);

        // Serial.print("Got servo 2: ");
        // Serial.print(theta2);
        // Serial.print("; Error: ");
        // Serial.println(-theta2 - servo2.getCurrentPosition());
        // Serial.println(servo2.getCurrentPosition());
        // Serial.print(" - ");

        servo1.setPositionInDeg(-theta1);
        servo2.setPositionInDeg(-theta2);
        servo3.setPositionInDeg(-theta3);

        // if (robot.hasFallen(customImu.getGravity())) {
        //     robot.stop();
        //     robotFell = true;
        //     Serial.println("Stopping motors because robot fell");
        // }
    }
}
