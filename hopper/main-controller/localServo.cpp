#include "localServo.h"
#define TCAADDR 0x70

void tcaselect(uint8_t i) {
    if (i > 7)
        return;

    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << i);
    Wire.endTransmission();
}

Servo::Servo(
    uint8_t PWM1,
    uint8_t PWM2,
    uint8_t OCM,
    uint8_t DIAG,
    uint8_t EN,
    uint8_t as5600Pin,
    int zeroPosition,
    bool direction) {
    this->PWM1 = PWM1;
    this->PWM2 = PWM2;
    this->OCM = OCM;
    this->DIAG = DIAG;
    this->EN = EN;
    this->as5600Pin = as5600Pin;
    this->zeroPosition = zeroPosition;
    this->direction = direction;
}

void Servo::begin() {
    // Setup gear motor
    pinMode(this->PWM1, OUTPUT);
    pinMode(this->PWM2, OUTPUT);
    pinMode(this->OCM, INPUT);
    pinMode(this->DIAG, INPUT);
    pinMode(this->EN, OUTPUT);
    digitalWrite(this->PWM1, LOW);
    digitalWrite(this->PWM2, LOW);

    tcaselect(this->as5600Pin);
    // Setup encoder
    as5600.begin(4);                        //  set direction pin.
    as5600.setDirection(AS5600_CLOCK_WISE); // default, just be explicit.

    int b = as5600.isConnected();

    if (as5600.readAngle() > this->zeroPosition)
        this->currentTurn--;
};

int Servo::getCurrentPosition() {
    tcaselect(this->as5600Pin);
    int encoderAngle = as5600.readAngle();

    // Check if there was a turn
    if (this->previousPosition > 4000 && encoderAngle < 100)
        this->currentTurn++;

    if (this->previousPosition < 100 && encoderAngle > 4000)
        this->currentTurn--;

    this->previousPosition = encoderAngle;
    int result = direction
                     ? (this->currentTurn * 4096 + encoderAngle) - zeroPosition
                     : zeroPosition - (this->currentTurn * 4096 + encoderAngle);

    return result * 360.0 / 4096.0;
}

float Servo::getPIDOutput(float error) {
    float kp = 5.75; // Pc ~140?
    float kd = 0.0;
    float ki = 0.0;

    float deltaError = (this->previousError - error) / this->deltaTime;

    float output = kp * error + kp * deltaError + ki * this->integralError;

    return output;
}

void Servo::setPositionInDeg(float desiredPosition) {
    // Protect robot
    if (desiredPosition > -5.0 || desiredPosition < -130.0)
        desiredPosition = -30;

    this->deltaTime = millis() - previousPositionTime;
    int currentPosition = this->getCurrentPosition();

    float error = desiredPosition - currentPosition;
    this->integralError += error * this->deltaTime;

    float output = this->getPIDOutput(error);

    setMotorTorque(output);

    this->previousPositionTime = millis();
    this->previousError = error;
};

void Servo::setMotorTorque(float speed) {
    // The level at which the motor starts moving
    float zeroSpeed = 0.0 / 5.0 * 255.0;
    float maxSpeed = 255.0;

    float adjustedSpeed = speed == 0 ? 0.0 : abs(speed) + zeroSpeed;

    if (adjustedSpeed > maxSpeed)
        adjustedSpeed = maxSpeed;

    if (speed > 0) {
        digitalWrite(this->EN, 1);
        analogWrite(this->PWM1, 0);
        analogWrite(this->PWM2, round(adjustedSpeed));
    } else {
        digitalWrite(this->EN, 1);
        analogWrite(this->PWM1, round(adjustedSpeed));
        analogWrite(this->PWM2, 0);
    }
}

void Servo::torqueOff() {
    digitalWrite(this->PWM1, LOW);
    digitalWrite(this->PWM2, LOW);
    digitalWrite(this->EN, LOW);
}
