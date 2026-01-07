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


const int REACTION_MOTOR_Y_PWM1 = 0, REACTION_MOTOR_Y_PWM2 = 2, REACTION_MOTOR_Y_DIAG = 4,
          REACTION_MOTOR_Y_EN = 6;
const int REACTION_MOTOR_X_PWM1 = 1, REACTION_MOTOR_X_PWM2 = 3, REACTION_MOTOR_X_DIAG = 5,
          REACTION_MOTOR_X_EN = 7;

// torque between 0 and 255
void setMotorTorqueX(int torque, bool direction) {
    digitalWrite(REACTION_MOTOR_X_EN, HIGH);
    if (direction) {
        digitalWrite(REACTION_MOTOR_X_PWM1, 0);
        analogWrite(REACTION_MOTOR_X_PWM2, torque);
    } else {
        analogWrite(REACTION_MOTOR_X_PWM1, torque);
        digitalWrite(REACTION_MOTOR_X_PWM2, 0);
    }
}


void setMotorTorqueY(int torque, bool direction) {
    digitalWrite(REACTION_MOTOR_Y_EN, HIGH);
    if (direction) {
        digitalWrite(REACTION_MOTOR_Y_PWM1, 0);
        analogWrite(REACTION_MOTOR_Y_PWM2, torque);
    } else {
        analogWrite(REACTION_MOTOR_Y_PWM1, torque);
        digitalWrite(REACTION_MOTOR_Y_PWM2, 0);
    }
}

void setupMotors()
{
    delay(5000);
    Serial.begin(115200);
    Serial.write("DONE world\n");

    // SETUP Motor Y
    // Disable Motor
    pinMode(REACTION_MOTOR_X_EN, OUTPUT);
    digitalWrite(REACTION_MOTOR_X_EN, LOW);

    pinMode(REACTION_MOTOR_X_PWM1, OUTPUT);
    pinMode(REACTION_MOTOR_X_PWM2, OUTPUT);
    // pinMode(REACTION_MOTOR_X_OCM, INPUT);
    // pinMode(REACTION_MOTOR_X_DIAG, INPUT);
    digitalWrite(REACTION_MOTOR_X_PWM1, LOW);
    digitalWrite(REACTION_MOTOR_X_PWM2, LOW);

    // SETUP Motor Y

    pinMode(REACTION_MOTOR_Y_EN, OUTPUT);
    digitalWrite(REACTION_MOTOR_Y_EN, LOW);

    pinMode(REACTION_MOTOR_Y_PWM1, OUTPUT);
    pinMode(REACTION_MOTOR_Y_PWM2, OUTPUT);
    // pinMode(REACTION_MOTOR_Y_OCM, INPUT);
    // pinMode(REACTION_MOTOR_Y_DIAG, INPUT);
    digitalWrite(REACTION_MOTOR_Y_PWM1, LOW);
    digitalWrite(REACTION_MOTOR_Y_PWM2, LOW);

    Serial.println("Setup complete\n");
}


void setup()
{
    delay(5000);
    Serial.begin(115200);
    Serial.write("Starting...");

    setupMotors();

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
    setMotorTorqueX(50, true);
    setMotorTorqueY(50, true);

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
