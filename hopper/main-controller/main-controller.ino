// #include "InverseKinematics.h"
// #include "imu.h"
// #include "leg.h"
#include "servo.h"
// #include "robot.h"
// #include "spiServo.h"

const int SER1_PWM1 = 5, SER1_PWM2 = 6, SER1_OCM = 26, SER1_DIAG = 4,
          SER1_EN = 7;
const int SER2_PWM1 = 9, SER2_PWM2 = 10, SER2_OCM = 27, SER2_DIAG = 8,
          SER2_EN = 11;
const int SER3_PWM1 = 21, SER3_PWM2 = 20, SER3_OCM = 26, SER3_DIAG = 22,
          SER3_EN = 19;
// const int SER1_AND_3_CHIP_SELECT_PIN = 10;

// IMU customImu = IMU();

Servo servo1 =
    Servo(SER1_PWM1, SER1_PWM2, SER1_OCM, SER1_DIAG, SER1_EN, 326, true);
Servo servo2 =
    Servo(SER2_PWM1, SER2_PWM2, SER2_OCM, SER2_DIAG, SER2_EN, 326, true);
Servo servo3 =
    Servo(SER3_PWM1, SER3_PWM2, SER3_OCM, SER3_DIAG, SER3_EN, 326, true);

// Leg leg = Leg(&servo1, &servo2, &servo3);
// Robot robot = Robot(&leg);

void setup() {
    Serial.begin(115200);
    Serial.println("Setup Done");

    // robot.begin();
    // customImu.begin();
    servo1.begin();
    servo2.begin();
    servo3.begin();
    // servo2.torqueOn();

    // Wire.setClock(1000000);

    // leg.setFootPosition(0, 0, -60);

    Serial.println("Setup Done");
}

long lastSecond = 0;
long loops = 0;
long lastMillisIMU = 0;
bool robotFell = false;
// Vector bodyOrientation;
// Vector linearAcceleration;
// Vector linearVelocity;
// Vector angularVelocity;

void loop() {
    loops++;
    // Serial.print("-0");
    long currTime = millis();

    if (lastSecond + 1000 < currTime) {
        lastSecond = currTime;
        Serial.print("Current hz: ");
        Serial.println(loops);
        loops = 0;
    }

    servo1.setMotorTorque(120);
    servo2.setMotorTorque(120);
    servo3.setMotorTorque(120);

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

        // Serial.print("0");
        // robot.updateStateIfChanged();
        // if (robot.getCurrentState() == STANCE_GOING_DOWN ||
        //     robot.getCurrentState() == STANCE_GOING_UP) {

        // Serial.print("1");
        // robot.sendCommandsToDuringStance(bodyOrientation.x, bodyOrientation.y);
        // } else {

        // Serial.print("1");
        // robot.sendCommandsToMotorsDuringFlight(
        //     linearVelocity.x,
        //     linearVelocity.y,
        //     bodyOrientation.x,
        //     bodyOrientation.y,
        //     angularVelocity.x,
        //     angularVelocity.y);
        // }

        // Serial.print("Got orientation - x: ");
        // Serial.print(bodyOrientation.x);
        // Serial.print("; y: ");
        // Serial.print(bodyOrientation.y);
        // Serial.println();

        // Serial.print("Touching ground? ");
        // Serial.print(leg.isFootTouchingGround());
        // Serial.println();
        // leg.isFootTouchingGround();
        // float val1 = -25.0 * sin(millis() / 100.0);
        // float val1 = -100.0 - 25.0 * sin(millis() / 1000.0);

        // float theta1;
        // float theta2;
        // float theta3;

        // leg.setDesiredAlphaXYInDeg(0, 0, bodyOrientation.x, 0);

        // int status = delta_calcInverse(30, 0, -100, theta1, theta2, theta3);

        // Serial.print("Got servo 2: ");
        // Serial.print(theta2);
        // Serial.print("; Error: ");
        // Serial.println(-theta2 - servo2.getCurrentPosition());
        // Serial.println(servo2.getCurrentPosition());
        // Serial.print(" - ");

        // servo1.setPositionInDeg(-theta1);
        // servo2.setPositionInDeg(-theta2);
        // servo3.setPositionInDeg(-theta3);

        // if (robot.hasFallen(customImu.getGravity())) {
        //     robot.stop();
        //     robotFell = true;
        //     Serial.println("Stopping motors because robot fell");
        // }
    // }
}
