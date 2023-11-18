#include "InverseKinematics.h"
#include "imu.h"
#include "leg.h"
#include "localServo.h"
#include "robot.h"
#include "spiServo.h"

const int SER1_PWM1 = 3, SER1_PWM2 = 5, SER1_OCM = A0, SER1_DIAG = 7,
          SER1_EN = 8, SER1_AS5600_MULTIPLEXER_PIN = -1;
const int SER2_AND_3_CHIP_SELECT_PIN = 10;

IMU customImu = IMU();

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
Robot robot = Robot(&leg);

void setup() {
    Serial.begin(115200);
    Serial.println("Setup Done");

    robot.begin();
    customImu.begin();

    leg.setPosition(0, 0, -100);
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

    leg.setPosition(40, 0, -100);
    // int pos = servo1.getCurrentPosition();

    // Serial.print("positions: ");
    // Serial.println(pos);

    // if (!robotFell) {
    //     if (lastMillisIMU + 50 < currTime) {
    //         lastMillisIMU = currTime;
    //         bool isFootTouchingGround = leg.isFootTouchingGround();

    //         if (isFootTouchingGround) {
    //             bodyOrientation = customImu.getOrientation();
    //         } else {
    //             bodyOrientation = customImu.getOrientation();
    //             linearVelocity = customImu.getComputedLinearVelocity();
    //             angularVelocity = customImu.getAngularVelocity();
    //         }
    //     }

    //     robot.updateStateIfChanged();
    //     if (robot.getCurrentState() == STANCE_GOING_DOWN ||
    //         robot.getCurrentState() == STANCE_GOING_UP) {
    //         robot.sendCommandsToDuringStance(
    //             bodyOrientation.x, bodyOrientation.y);
    //     } else {

    //         robot.sendCommandsToMotorsDuringFlight(
    //             linearVelocity.x,
    //             linearVelocity.y,
    //             bodyOrientation.x,
    //             bodyOrientation.y,
    //             angularVelocity.x,
    //             angularVelocity.y);
    //     }

    //     // Serial.print("Got orientation - x: ");
    //     // Serial.print(bodyOrientation.x);
    //     // Serial.print("; y: ");
    //     // Serial.print(bodyOrientation.y);
    //     // Serial.println();

    //     // float zValue = -100.0 - 25.0 * sin(millis() / 100.0);

    //     // float theta1;
    //     // float theta2;
    //     // float theta3;

    //     // int status = delta_calcInverse(
    //     //     0, 0, static_cast<int>(zValue), theta1, theta2, theta3);

    //     // Serial.print("Got servo 2: ");
    //     // Serial.println(-theta1);

    //     // servo1.setPositionInDeg(-theta1);
    //     // servo2.setPositionInDeg(-theta2);
    //     // servo3.setPositionInDeg(-theta3);

    //     if (robot.hasFallen(customImu.getGravity())) {
    //         robot.stop();
    //         robotFell = true;
    //         Serial.println("Stopping motors because robot fell");
    //     }
    // }
}
