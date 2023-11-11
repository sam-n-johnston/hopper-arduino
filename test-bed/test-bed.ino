#include "InverseKinematics.h"
#include "Servo.h"

const int SER1_PWM1 = 11, SER1_PWM2 = 10, SER1_OCM = A0, SER1_DIAG = 8,
          SER1_EN = 7, SER1_AS5600_MULTIPLEXER_PIN = 2;
const int SER2_PWM1 = 9, SER2_PWM2 = 6, SER2_OCM = A1, SER2_DIAG = 4,
          SER2_EN = 2, SER2_AS5600_MULTIPLEXER_PIN = 4;
const int SER3_PWM1 = 5, SER3_PWM2 = 3, SER3_OCM = A2, SER3_DIAG = A3,
          SER3_EN = 12, SER3_AS5600_MULTIPLEXER_PIN = 6;

Servo servo1 = Servo(
    SER1_PWM1,
    SER1_PWM2,
    SER1_OCM,
    SER1_DIAG,
    SER1_EN,
    SER1_AS5600_MULTIPLEXER_PIN,
    1360,
    true);
Servo servo2 = Servo(
    SER2_PWM1,
    SER2_PWM2,
    SER2_OCM,
    SER2_DIAG,
    SER2_EN,
    SER2_AS5600_MULTIPLEXER_PIN,
    930,
    true);
Servo servo3 = Servo(
    SER3_PWM1,
    SER3_PWM2,
    SER3_OCM,
    SER3_DIAG,
    SER3_EN,
    SER3_AS5600_MULTIPLEXER_PIN,
    2220,
    true);

void setup() {
    Serial.begin(115200);
    servo1.begin();
    servo2.begin();
    servo3.begin();
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

    int timeInterval = 2000;
    float zValue = -100.0 - 25.0 * sin(millis() / 100.0);

    int status = delta_calcInverse(
        0, 0, static_cast<int>(zValue), theta1, theta2, theta3);

    servo1.setPositionInDeg(-theta1);
    servo2.setPositionInDeg(-theta2);
    servo3.setPositionInDeg(-theta3);
}
