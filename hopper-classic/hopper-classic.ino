#include <Servo.h>
#include "imu.h"
#include "leg.h"
#include "puller.h"
#include "robot.h"

const int SER1_PWM1 = 5, SER1_PWM2 = 6, SER1_OCM = 26, SER1_DIAG = 4,
          SER1_PLEX = 0, SER1_EN = 7;

IMU customImu = IMU();

Puller puller =
    Puller(SER1_PWM1, SER1_PWM2, SER1_OCM, SER1_DIAG, SER1_EN, SER1_PLEX, 1650, true);

Servo servoX;
Servo servoY;
Leg leg = Leg(&puller, &servoX, &servoY);
Robot robot = Robot(&leg);

// float position1 = 0.0;
// float position2 = 0.0;
// float position3 = 0.0;
bool setupDone = false;
bool robotFell = false;
// long lastSecond = 0;
long lastSecond1 = 0;
// long loops = 0;
long loops1 = 0;
long lastMillisIMU = 0;
Vector bodyOrientation;
Vector linearAcceleration;
Vector linearVelocity;
Vector angularVelocity;

void setup1()
{
    delay(5000);
    Serial.begin(115200);

    // For the AS5600
    bool test2 = Wire1.setSDA(14);
    bool test1 = Wire1.setSCL(15);

    // For the IMU
    bool test3 = Wire.setSDA(16);
    bool test4 = Wire.setSCL(17);

    if (!test1 || !test2 || !test3 || !test4) {
        Serial.print("Failed to set SDA/SCL: ");
        Serial.print(test1);
        Serial.print(test2);
        Serial.print(test3);
        Serial.print(test4);
        Serial.println();
    }

    customImu.begin();
    robot.begin();
    Serial.println("Starting Core1 5");
    setupDone = true;
}

void loop1()
{
    loops1++;
    long currTime = millis();

    if (lastSecond1 + 1000 < currTime)
    {
        lastSecond1 = currTime;
        Serial.print("Current hz (core 1): ");
        Serial.println(loops1);
        loops1 = 0;
    }
    // leg.setDesiredAlphaXYInDeg(90, 90);

    // bool isFootTouchingGround = leg.isFootTouchingGround();

    // if (lastMillisIMU + 50 < currTime) {
    //     lastMillisIMU = currTime;
        customImu.getSensorData();
    // }

    if (!robotFell)
    {
        // bool isFootTouchingGround = leg.isFootTouchingGround();

        if (false)
        {
            bodyOrientation = customImu.getOrientation();
        }
        else
        {
            bodyOrientation = customImu.getOrientation();
            linearVelocity = customImu.getComputedLinearVelocity();
            // angularVelocity = customImu.getAngularVelocity();
        }

        // robot.updateStateIfChanged();
        // if (robot.getCurrentState() == STANCE_GOING_DOWN ||
        //     robot.getCurrentState() == STANCE_GOING_UP)
        // {
        //     robot.sendCommandsToDuringStance(
        //         bodyOrientation.x,
        //         bodyOrientation.y);
        // }
        // else
        // {
            robot.sendCommandsToMotorsDuringFlight(
                linearVelocity.x,
                linearVelocity.y,
                bodyOrientation.x,
                bodyOrientation.y,
                angularVelocity.x,
                angularVelocity.y);
        // }

        // if (robot.hasFallen(customImu.getGravity()))
        // {
        //     Serial.println("Robot fell!");
        //     robot.stop();
        //     robotFell = true;
        //     Serial.println("Stopping motors because robot fell");
        // }
    }
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Starting Core0");
    while (!setupDone) {
        delay(10);
    }
    Serial.println("Starting Core0 program");
}

void loop()
{
//     loops++;
//     long currTime = millis();

//     if (lastSecond + 1000 < currTime)
//     {
//         lastSecond = currTime;
//         Serial.print("Current hz (core 0): ");
//         Serial.println(loops);
//         loops = 0;
//     }

//     puller.goToDesiredPosition();
}
