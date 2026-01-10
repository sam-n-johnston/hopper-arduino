#include <Dynamixel2Arduino.h>
#include "imu.h"

UART Serial2(8, 9, 0, 0);
const int DXL_DIR_PIN = 28; // DYNAMIXEL Shield DIR PIN - Dummy pin here
const float DXL_PROTOCOL_VERSION = 2.0;
const int X_SERVO_ID = 1;
const int Y_SERVO_ID = 2;

IMU customImu = IMU();
Vector bodyOrientation;

Dynamixel2Arduino dlx;

// --- Reaction wheel control state ---
const unsigned long CONTROL_INTERVAL_US = 1000; // 50 Hz control loop
unsigned long lastControlMicros = 0;
float timeDelta = 0;
const unsigned long SERVO_UPDATE_INTERVAL_US = 200000; // 5 Hz servo update
unsigned long lastServoMicros = 0;

// PID gains (initial values, tune on hardware)
float pidX_kp = 3.0f, pidX_ki = 0.0f, pidX_kd = 0.02f;
float pidY_kp = 3.0f, pidY_ki = 0.0f, pidY_kd = 0.02f;

float pidX_integral = 0.0f, pidX_prevError = 0.0f;
float pidY_integral = 0.0f, pidY_prevError = 0.0f;

int maxTorquePWM = 220; // cap torque PWM

bool reactionControlEnabled = true;

void logVector(Vector vec)
{
    // Serial.print(vec.x);
    // Serial.print(",");
    // Serial.print(vec.y);
    // Serial.print(",");
    // Serial.print(vec.z);
}

void readIMUrates()
{
    customImu.getSensorData();
    bodyOrientation = customImu.getOrientation();
    logVector(bodyOrientation);
    Serial.println();
}

// Simple PID that returns a signed control value (mapped to PWM units)
float pidUpdate(float setpoint, float measurement, float &integral, float &prevError,
                float kp, float ki, float kd, float dt)
{
    float error = setpoint - measurement;
    integral += error * dt;
    // anti-windup: clamp integral
    const float maxIntegral = 1000.0f;
    if (integral > maxIntegral) integral = maxIntegral;
    if (integral < -maxIntegral) integral = -maxIntegral;
    float derivative = 0.0f;
    if (dt > 0.0f) derivative = (error - prevError) / dt;
    prevError = error;
    float out = kp * error + ki * integral + kd * derivative;
    return out;
}

// Apply signed control value to motors (convert to PWM + direction)
void applyReactionControl(float controlX, float controlY)
{
    if (!reactionControlEnabled)
    {
        setMotorTorqueX(0, false);
        setMotorTorqueY(0, false);
        return;
    }

    int pwmX = (int)fabs(controlX);
    if (pwmX > maxTorquePWM) pwmX = maxTorquePWM;
    bool dirX = controlX > 0.0f; // positive control -> spin in "true" direction
    setMotorTorqueX(pwmX, dirX);

    int pwmY = (int)fabs(controlY);
    if (pwmY > maxTorquePWM) pwmY = maxTorquePWM;
    bool dirY = controlY > 0.0f;
    setMotorTorqueY(pwmY, dirY);
}

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
    // Serial.write("Starting...\n");

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

    // Serial.println("Setup complete\n");
}


void setup()
{
    delay(5000);
    Serial.begin(115200);
    // Serial.write("Starting...");

    setupMotors();
    customImu.begin();

    // initialize control timing
    lastControlMicros = micros();
    lastServoMicros = micros();

    dlx = Dynamixel2Arduino(Serial2, DXL_DIR_PIN);
    dlx.begin(57600);
    delay(50);
    dlx.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

    // Ping the X servo to check connection
    bool pingRes = dlx.ping(X_SERVO_ID);
    delay(50);
    // Serial.println();
    // Serial.print("Ping Servo X: ");
    // Serial.print(pingRes);

    // Ping the Y servo to check connection
    pingRes = dlx.ping(Y_SERVO_ID);
    delay(50);
    // Serial.println();
    // Serial.print("Ping Servo Y: ");
    // Serial.print(pingRes);

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

    // Serial.println();
    // Serial.println("Setup complete");
    Serial.print("gyro:,body:,filtered:");
    Serial.println();
}

void loop()
{
    unsigned long now = micros();

    // Control loop (run at CONTROL_INTERVAL_US)
    if ((now - lastControlMicros) >= CONTROL_INTERVAL_US)
    {
        timeDelta = (now - lastControlMicros) / 1000000.0f; // seconds
        lastControlMicros = now;

        // Read IMU and update filtered rates
        readIMUrates();

        // Safety: if orientation is extreme, disable control
        if (fabs(bodyOrientation.x) > 60.0f || fabs(bodyOrientation.y) > 60.0f)
        {
            reactionControlEnabled = false;
            Serial.println("Safety: orientation exceeded limit, disabling reaction control");
            setMotorTorqueX(0, false);
            setMotorTorqueY(0, false);
        }

        // Compute PID outputs (setpoint = 0 deg/s)
        float controlX = pidUpdate(0.0f, bodyOrientation.x, pidX_integral, pidX_prevError,
                                   pidX_kp, pidX_ki, pidX_kd, timeDelta);
        float controlY = pidUpdate(0.0f, bodyOrientation.y, pidY_integral, pidY_prevError,
                                   pidY_kp, pidY_ki, pidY_kd, timeDelta);

        // Map PID output to PWM range and apply
        // The PID output is interpreted as PWM magnitude; tune kp/ki/kd accordingly.
        // applyReactionControl(controlX, controlY);

        // debug print occasionally
        static unsigned long lastDebugMillis = 0;
        if (millis() - lastDebugMillis > 1000)
        {
            lastDebugMillis = millis();
            // Serial.print("Rates (deg/s) X:");
            // Serial.print(filteredRateX);
            // Serial.print(" Y:");
            // Serial.print(filteredRateY);
            // Serial.print(" -> PWM X:");
            // Serial.print((int)fabs(controlX));
            // Serial.print(" Y:");
            // Serial.println((int)fabs(controlY));
        }
    }

    // Servo updates at lower frequency to avoid blocking the control loop
    if ((now - lastServoMicros) >= SERVO_UPDATE_INTERVAL_US)
    {
        lastServoMicros = now;

        int present_position_x = dlx.getPresentPosition(X_SERVO_ID);
        delay(10);
        int present_position_y = dlx.getPresentPosition(Y_SERVO_ID);
        delay(10);

        float desiredDegreeAngle = 180.0;
        float desiredRawAngle = desiredDegreeAngle / 360.0 * 4096.0;

        dlx.setGoalPosition(X_SERVO_ID, desiredRawAngle);
        delay(10);
        dlx.setGoalPosition(Y_SERVO_ID, desiredRawAngle);
        delay(10);
    }
}
