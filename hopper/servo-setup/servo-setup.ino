#include "servo.h"

/**
 * @brief Run this to update the `zeroPosition` of the servo 
 * Use [Measurement] - 30/360 * 4096
 */

const int SER1_PWM1 = 5, SER1_PWM2 = 6, SER1_OCM = 26, SER1_DIAG = 4,
          SER1_PLEX = 0, SER1_EN = 7;
const int SER2_PWM1 = 9, SER2_PWM2 = 10, SER2_OCM = 27, SER2_DIAG = 8,
          SER2_PLEX = 3, SER2_EN = 11;
const int SER3_PWM1 = 21, SER3_PWM2 = 20, SER3_OCM = 26, SER3_DIAG = 22,
          SER3_PLEX = 2, SER3_EN = 19;

Servo servo1 =
    Servo(SER1_PWM1, SER1_PWM2, SER1_OCM, SER1_DIAG, SER1_EN, SER1_PLEX, 1419, true);
Servo servo2 =
    Servo(SER2_PWM1, SER2_PWM2, SER2_OCM, SER2_DIAG, SER2_EN, SER2_PLEX, 338, true);
Servo servo3 =
    Servo(SER3_PWM1, SER3_PWM2, SER3_OCM, SER3_DIAG, SER3_EN, SER3_PLEX, 800, true);

long lastSecond = 0;
long loops = 0;

void setup()
{
    delay(5000);
    Serial.begin(115200);
    Serial.println("Starting Core0");

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

    servo1.begin();
    servo2.begin();
    servo3.begin();
    // servo1.torqueOn();
    // servo2.torqueOn();
    // servo3.torqueOn();
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

    float pos1 = servo1.getCurrentPositionRaw();
    float pos2 = servo2.getCurrentPositionRaw();
    float pos3 = servo3.getCurrentPositionRaw();

    Serial.print("Positions:");
    Serial.print(pos1);
    Serial.print(" - pos2: ");
    Serial.print(pos2);
    Serial.print(" - pos3: ");
    Serial.print(pos3);
    Serial.println(";");
}
