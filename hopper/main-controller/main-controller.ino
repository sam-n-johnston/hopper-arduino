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
    LocalServo(SER2_PWM1, SER2_PWM2, SER2_OCM, SER2_DIAG, SER2_EN, 326, true);
SPIServo servo3 = SPIServo(
    SER1_AND_3_CHIP_SELECT_PIN,
    QUERY_GET_POSITION3,
    COMMAND_SET_GOAL_POSITION3,
    TORQUE_OFF3,
    TORQUE_ON3);

Leg leg = Leg(&servo1, &servo2, &servo3);
Robot robot = Robot(&leg);

int size = 175;
long millisArray[175];
int positionsArray[175];

void setup() {
    Serial.begin(115200);
    Serial.println("Setup Done");

    // robot.begin();
    servo2.begin();
    customImu.begin();

    Wire.setClock(1000000);

    // leg.setFootPosition(0, 0, -75);

    Serial.println("Setup Done");
}

long lastSecond = 0;
long loops = 0;
long lastMillisIMU = 0;
int counter = 0;
bool printed = false;
bool robotFell = false;
// Vector bodyOrientation;
// Vector linearAcceleration;
// Vector linearVelocity;
// Vector angularVelocity;

void loop() {
    loops++;
    long currTime = millis();

    if (lastSecond + 1000 < currTime) {
        lastSecond = currTime;
        Serial.print("Current hz: ");
        Serial.println(loops);
        loops = 0;
    }

    long millisCurr = millis();

    if (millisCurr < 3000) {
        servo2.setPositionInDeg(0);
    } else if (millisCurr < 3060) {
        if (counter < size) {
            millisArray[counter] = millisCurr;
            positionsArray[counter] = servo2.getCurrentPosition();
            counter++;
        }
        servo2.setMotorTorque(-255.0);
        delay(1);
        // servo2.setPositionInDeg(-45);
    } else {
        if (!printed) {
            for (int i = 0; i < size; i++) {
                Serial.print(millisArray[i]);
                Serial.print(",");
                Serial.print(positionsArray[i]);
                Serial.println();
            }

            printed = true;
        }

        servo2.torqueOff();
    }
}
