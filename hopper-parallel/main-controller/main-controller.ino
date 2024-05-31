#include "InverseKinematics.h"
#include "imu.h"
#include "leg.h"
#include "servo.h"
#include "robot.h"

const int SER1_PWM1 = 5, SER1_PWM2 = 6, SER1_OCM = 26, SER1_DIAG = 4,
          SER1_PLEX = 0, SER1_EN = 7;
const int SER2_PWM1 = 9, SER2_PWM2 = 10, SER2_OCM = 27, SER2_DIAG = 8,
          SER2_PLEX = 3, SER2_EN = 11;
const int SER3_PWM1 = 21, SER3_PWM2 = 20, SER3_OCM = 26, SER3_DIAG = 22,
          SER3_PLEX = 2, SER3_EN = 19;
// const int SER1_AND_3_CHIP_SELECT_PIN = 10;

IMU customImu = IMU();

Servo servo1 =
    Servo(SER1_PWM1, SER1_PWM2, SER1_OCM, SER1_DIAG, SER1_EN, SER1_PLEX, 1650, true);
Servo servo2 =
    Servo(SER2_PWM1, SER2_PWM2, SER2_OCM, SER2_DIAG, SER2_EN, SER2_PLEX, 338, true);
Servo servo3 =
    Servo(SER3_PWM1, SER3_PWM2, SER3_OCM, SER3_DIAG, SER3_EN, SER3_PLEX, 800, true);

Leg leg = Leg(&servo1, &servo2, &servo3);
Robot robot = Robot(&leg);

float position1 = 0.0;
float position2 = 0.0;
float position3 = 0.0;
bool setupDone = false;
bool robotFell = false;
long lastSecond = 0;
long lastSecond1 = 0;
long loops = 0;
long loops1 = 0;
// long lastMillisIMU = 0;
Vector bodyOrientation;
Vector linearAcceleration;
Vector linearVelocity;
Vector angularVelocity;

void setup1()
{
    delay(5000);
    Serial.begin(115200);
    Serial.println("Starting Core1");

    bool test1 = Wire.setSCL(1);
    bool test2 = Wire.setSDA(0);

    // bool test3 = Wire1.setSDA(2);
    // bool test4 = Wire1.setSCL(3);

    bool test3 = Wire1.setSDA(14);
    bool test4 = Wire1.setSCL(15);

    if (!test1 || !test2 || !test3 || !test4) {
        Serial.print("Failed to set SDA/SCL: ");
        Serial.print(test1);
        Serial.print(test2);
        Serial.print(test3);
        Serial.print(test4);
        Serial.println();
    }

    Wire.begin();
    Wire1.begin();

    customImu.begin();
    robot.begin();
    leg.setFootPosition(0.0, 0.0, -85.0);
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
    // bool isFootTouchingGround = leg.isFootTouchingGround();
    // bodyOrientation = customImu.getOrientation();

    // leg.setFootPosition(0.0, 0.0, -100.0);

    // if (lastMillisIMU + 50 < currTime) {
    //     lastMillisIMU = currTime;
    customImu.getSensorData();
    // }

    if (!robotFell)
    {
        bool isFootTouchingGround = leg.isFootTouchingGround();

        if (isFootTouchingGround)
        {
            bodyOrientation = customImu.getOrientation();
        }
        else
        {
            bodyOrientation = customImu.getOrientation();
            linearVelocity = customImu.getComputedLinearVelocity();
            angularVelocity = customImu.getAngularVelocity();
        }

        robot.updateStateIfChanged();
        if (robot.getCurrentState() == STANCE_GOING_DOWN ||
            robot.getCurrentState() == STANCE_GOING_UP)
        {
            robot.sendCommandsToDuringStance(
                bodyOrientation.x,
                bodyOrientation.y);
        }
        else
        {
            robot.sendCommandsToMotorsDuringFlight(
                linearVelocity.x,
                linearVelocity.y,
                bodyOrientation.x,
                bodyOrientation.y,
                angularVelocity.x,
                angularVelocity.y);
        }

        if (robot.hasFallen(customImu.getGravity()))
        {
            Serial.println("Robot fell!");
            robot.stop();
            robotFell = true;
            Serial.println("Stopping motors because robot fell");
        }
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
    loops++;
    // Serial.print("-0");
    long currTime = millis();

    if (lastSecond + 1000 < currTime)
    {
        lastSecond = currTime;
        Serial.print("Current hz (core 0): ");
        Serial.println(loops);
        loops = 0;
    }

    servo1.goToDesiredPosition();
    servo2.goToDesiredPosition();
    servo3.goToDesiredPosition();

    // float pos1 = servo1.getCurrentPosition();
    // float pos2 = servo2.getCurrentPosition();
    // float pos3 = servo3.getCurrentPosition();

    // Serial.print("Positions:");
    // Serial.print(pos1);
    // Serial.print(" - pos2: ");
    // Serial.print(pos2);
    // Serial.print(" - pos3: ");
    // Serial.print(pos3);
    // Serial.println(";");
}
