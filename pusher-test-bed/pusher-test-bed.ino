#include "leg.h"
#include "puller.h"
#include "robot.h"

const int PULLER_PWM1 = 12, PULLER_PWM2 = 11, PULLER_OCM = 26, PULLER_DIAG = 13,
          PULLER_EN = 10, PULLER_ZERO_POSITION = 2100;

Puller puller =
    Puller(PULLER_PWM1, PULLER_PWM2, PULLER_OCM, PULLER_DIAG, PULLER_EN, true, PULLER_ZERO_POSITION);

Leg leg = Leg(&puller);
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
    Serial.println("Starting Core1");

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

    // if (currTime < 10000 ) {
    //     puller.setPositionInDeg(0);
    // } else {
    //     puller.setPositionInDeg(0);
    // }

    // puller.getCurrentPosition();

    robot.updateStateIfChanged();
    if (robot.getCurrentState() == STANCE_GOING_DOWN ||
        robot.getCurrentState() == STANCE_GOING_UP)
    {
        robot.sendCommandsToDuringStance();
    }
    else
    {
        robot.sendCommandsToMotorsDuringFlight();
    }
}

void setup()
{
    delay(4900);
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
