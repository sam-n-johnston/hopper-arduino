#include <Dynamixel2Arduino.h>

UART Serial2(8, 9, 0, 0);
const int DXL_DIR_PIN = 28; // DYNAMIXEL Shield DIR PIN - Dummy pin here
const float DXL_PROTOCOL_VERSION = 2.0;
const int SERVO_ID = 1;

Dynamixel2Arduino dxl;

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

    dxl = Dynamixel2Arduino(Serial2, DXL_DIR_PIN);
    dxl.begin(57600);
    delay(50);
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
    bool pingRes = dxl.ping(SERVO_ID);
    delay(50);
    Serial.print("Ping Servo X: ");
    Serial.print(pingRes);

    delay(50);
    dxl.torqueOff(SERVO_ID);
    delay(50);
    dxl.setOperatingMode(SERVO_ID, OP_POSITION);
    delay(50);
    dxl.torqueOn(SERVO_ID);

    Serial.println("Setup complete");
}

void loop()
{
    int present_position = dxl.getPresentPosition(SERVO_ID);

    float desiredDegreeAngle = 180.0;
    float desiredRawAngle = desiredDegreeAngle / 360.0 * 4096.0;

    dxl.setGoalPosition(SERVO_ID, desiredRawAngle);
}
