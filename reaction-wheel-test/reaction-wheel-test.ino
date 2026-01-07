#include <Dynamixel2Arduino.h>

UART Serial2(8, 9, 0, 0);
const int DXL_DIR_PIN = 28; // DYNAMIXEL Shield DIR PIN - Dummy pin here
const float DXL_PROTOCOL_VERSION = 2.0;
const int X_SERVO_ID = 1;
const int Y_SERVO_ID = 2;

Dynamixel2Arduino dlx;

void blink(int numberOfBlinks) {
    for (int i = 0; i < numberOfBlinks; i++) {
        digitalWrite(LED_BUILTIN, 1);
        delay(250);
        digitalWrite(LED_BUILTIN, 0);
        delay(250);
    }
}

void setup()
{
    delay(5000);
    Serial.begin(115200);
    Serial.write("DONE world");

    dlx = Dynamixel2Arduino(Serial2, DXL_DIR_PIN);
    dlx.begin(57600);
    delay(50);
    dlx.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

    // Ping the X servo to check connection
    bool pingRes = dlx.ping(X_SERVO_ID);
    delay(50);
    Serial.println();
    Serial.print("Ping Servo X: ");
    Serial.print(pingRes);

    // Ping the Y servo to check connection
    pingRes = dlx.ping(Y_SERVO_ID);
    delay(50);
    Serial.println();
    Serial.print("Ping Servo Y: ");
    Serial.print(pingRes);

    delay(50);
    dlx.torqueOff(X_SERVO_ID);
    delay(50);
    dlx.torqueOff(Y_SERVO_ID);
    delay(50);
    dlx.setOperatingMode(X_SERVO_ID, OP_POSITION);
    delay(50);
    dlx.setOperatingMode(Y_SERVO_ID, OP_POSITION);
    delay(50);
    dlx.torqueOn(X_SERVO_ID);
    delay(50);
    dlx.torqueOn(Y_SERVO_ID);

    Serial.println();
    Serial.println("Setup complete");
}

void loop()
{
    int present_position_x = dlx.getPresentPosition(X_SERVO_ID);
    delay(50);
    int present_position_y = dlx.getPresentPosition(Y_SERVO_ID);
    delay(50);

    float desiredDegreeAngle = 180.0;
    float desiredRawAngle = desiredDegreeAngle / 360.0 * 4096.0;

    dlx.setGoalPosition(X_SERVO_ID, desiredRawAngle);
    delay(50);
    dlx.setGoalPosition(Y_SERVO_ID, desiredRawAngle);
    delay(50);
}
