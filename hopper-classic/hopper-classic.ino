#include <Servo.h>
#include "imu.h"
#include "leg.h"
#include "puller.h"
#include "robot.h"

const int PULLER_PWM1 = 12, PULLER_PWM2 = 11, PULLER_OCM = 26, PULLER_DIAG = 13,
          PULLER_EN = 10;

IMU customImu = IMU();

Puller puller =
    Puller(PULLER_PWM1, PULLER_PWM2, PULLER_OCM, PULLER_DIAG, PULLER_EN, true);

Servo servoX;
Servo servoY;
Leg leg = Leg(&puller, &servoX, &servoY);
Robot robot = Robot(&leg);

bool setupDone = false;
bool robotFell = false;
long lastSecond = 0;
long lastSecond1 = 0;
long loops = 0;
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

    if (!test1 || !test2 || !test3 || !test4)
    {
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

    customImu.getSensorData();

    // puller.setPositionInDeg(-180);
    // puller.goToDesiredPosition();

    if (!robotFell)
    {
        bool isFootTouchingGround = leg.isFootTouchingGround();
        bodyOrientation = customImu.getOrientation();

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
        // robot.sendCommandsToMotorsDuringFlight(
        //     linearVelocity.x,
        //     linearVelocity.y,
        //     bodyOrientation.x,
        //     bodyOrientation.y,
        //     angularVelocity.x,
        //     angularVelocity.y);
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
    while (!setupDone)
    {
        delay(10);
    }
    Serial.println("Starting Core0 program");
}

void loop()
{
        loops++;
        long currTime = millis();

        if (lastSecond + 1000 < currTime)
        {
            lastSecond = currTime;
            Serial.print("Current hz (core 0): ");
            Serial.println(loops);
            loops = 0;
        }

        puller.goToDesiredPosition();
}
