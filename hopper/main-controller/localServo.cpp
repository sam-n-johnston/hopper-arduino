#include "localServo.h"

LocalServo::LocalServo(
    uint8_t PWM1,
    uint8_t PWM2,
    uint8_t OCM,
    uint8_t DIAG,
    uint8_t EN,
    int zeroPosition,
    bool direction) {
    this->PWM1 = PWM1;
    this->PWM2 = PWM2;
    this->OCM = OCM;
    this->DIAG = DIAG;
    this->EN = EN;
    this->zeroPosition = zeroPosition;
    this->direction = direction;
}

void LocalServo::begin() {
    // Setup gear motor
    pinMode(this->PWM1, OUTPUT);
    pinMode(this->PWM2, OUTPUT);
    pinMode(this->OCM, INPUT);
    pinMode(this->DIAG, INPUT);
    pinMode(this->EN, OUTPUT);
    digitalWrite(this->PWM1, LOW);
    digitalWrite(this->PWM2, LOW);

    // Setup encoder
    as5600.begin(4);                        //  set direction pin.
    as5600.setDirection(AS5600_CLOCK_WISE); // default, just be explicit.

    int b = as5600.isConnected();

    if (b == 0)
        Serial.print("AS5600 failed to connect!");

    Serial.print("Local Servo Setup Done");
};

float LocalServo::getCurrentPosition() {
    int encoderAngle = as5600.readAngle();

    // Check if there was a turn
    if (this->previousPosition > 4000 && encoderAngle < 100)
        this->currentTurn++;

    if (this->previousPosition < 100 && encoderAngle > 4000)
        this->currentTurn--;

    this->positionDelta = encoderAngle - this->previousPosition;
    this->previousPosition = encoderAngle;
    int result = direction
                     ? (this->currentTurn * 4096 + encoderAngle) - zeroPosition
                     : zeroPosition - (this->currentTurn * 4096 + encoderAngle);

    return result * 360.0 / 4096.0;
}

float LocalServo::getPIDOutput(float error) {
    float kp = 2.0; // Pc ~140?
    float kd = 0.0;
    float ki = 0.0;

    float deltaError = (this->previousError - error) / this->deltaTime / 1000.0;

    float output = kp * error + kd * deltaError + ki * this->integralError;

    return output;
}

void LocalServo::setPositionInDeg(float desiredPosition) {
    if (desiredPosition > 30.0 || desiredPosition < -90.0) {
        Serial.println("Tried to set position outside acceptable range");
        desiredPosition = 0;
    }

    unsigned long currentTime = micros();
    this->deltaTime = currentTime - previousPositionTime;

    float currentPosition = this->getCurrentPosition();

    float error = desiredPosition - currentPosition;
    this->integralError += error * this->deltaTime / 1000.0;

    float output = this->getPIDOutput(error);

    setMotorTorque(output);

    this->previousPositionTime = currentTime;
    this->previousError = error;
};

void LocalServo::setMotorTorque(float speed) {
    // The level at which the motor starts moving
    float zeroSpeed = 0.0;
    float maxSpeed = 255.0;

    float jolt = 0.0;

    if (this->positionDelta == 0) {
        jolt = 15.0;
    }

    float adjustedSpeed = speed == 0 ? 0.0 : abs(speed) + zeroSpeed + jolt;

    if (adjustedSpeed > maxSpeed)
        adjustedSpeed = maxSpeed;

    if (speed > 0) {
        digitalWrite(this->EN, 1);
        digitalWrite(this->PWM1, 0);
        analogWrite(this->PWM2, round(adjustedSpeed));
    } else {
        digitalWrite(this->EN, 1);
        analogWrite(this->PWM1, round(adjustedSpeed));
        digitalWrite(this->PWM2, 0);
    }
}

void LocalServo::torqueOff() {
    digitalWrite(this->PWM1, LOW);
    digitalWrite(this->PWM2, LOW);
    digitalWrite(this->EN, LOW);
}

void LocalServo::torqueOn() {
    digitalWrite(this->PWM1, LOW);
    digitalWrite(this->PWM2, LOW);
    digitalWrite(this->EN, HIGH);
}
