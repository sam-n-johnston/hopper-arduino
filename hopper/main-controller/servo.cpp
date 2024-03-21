#include "servo.h"
#define TCAADDR 0x70

void tcaselect(uint8_t i) {
    if (i > 7)
        return;

    Wire1.beginTransmission(TCAADDR);
    Wire1.write(1 << i);
    Wire1.endTransmission();
}

Servo::Servo(
    uint8_t PWM1,
    uint8_t PWM2,
    uint8_t OCM,
    uint8_t DIAG,
    uint8_t EN,
    uint8_t as5600MultiplexerPin,
    int zeroPosition,
    bool direction) {
    this->PWM1 = PWM1;
    this->PWM2 = PWM2;
    this->OCM = OCM;
    this->DIAG = DIAG;
    this->EN = EN;
    this->as5600MultiplexerPin = as5600MultiplexerPin;
    this->zeroPosition = zeroPosition;
    this->direction = direction;
}

void Servo::begin() {
    Serial.print("Setting up Servo... ");
    // Wire1.beginTransmission();
    // Setup gear motor
    pinMode(this->PWM1, OUTPUT);
    pinMode(this->PWM2, OUTPUT);
    pinMode(this->OCM, INPUT);
    pinMode(this->DIAG, INPUT);
    pinMode(this->EN, OUTPUT);
    digitalWrite(this->PWM1, LOW);
    digitalWrite(this->PWM2, LOW);

    tcaselect(this->as5600MultiplexerPin);
    this->as5600 = AS5600(&Wire1);
    
    // // // Setup encoder
    this->as5600.begin(4);                        //  set direction pin.
    this->as5600.setDirection(AS5600_CLOCK_WISE); // default, just be explicit.

    int b = as5600.isConnected();

    if (b == 0)
        Serial.print("- AS5600 failed to connect! - ");

    // float currentPos = this->getCurrentPosition();
    // if (currentPos > 35.0)
    //     this->currentTurn--;

    Serial.println("Done");
};

float Servo::getCurrentPosition() {
    tcaselect(this->as5600MultiplexerPin);
    int encoderAngle = this->as5600.readAngle();

    // Serial.print("Got this number: ");
    // Serial.println(encoderAngle);

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

float Servo::getPIDOutput(float error) {
    float kp = 2.0; // Pc ~140?
    float kd = 0.0;
    float ki = 0.0;

    float deltaError = (this->previousError - error) / this->deltaTime / 1000.0;

    float output = kp * error + kd * deltaError + ki * this->integralError;

    return output;
}

void Servo::setPositionInDeg(float desiredPosition) {
    if (desiredPosition > 30.0 || desiredPosition < -90.0) {
        if (desiredPosition > 30.0) {
            desiredPosition = 30.0;
        } else {
            desiredPosition = -90.0;
        }
    }

    unsigned long currentTime = micros();
    this->deltaTime = currentTime - previousPositionTime;

    float currentPosition = this->getCurrentPosition();

    float error = desiredPosition - currentPosition;
    float intError = error * this->deltaTime / 1000.0;
    if (intError == intError)
        this->integralError += intError;

    float output = this->getPIDOutput(error);

    setMotorTorque(output);

    this->previousPositionTime = currentTime;
    this->previousError = error;
};

void Servo::setMotorTorque(float speed) {
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

void Servo::torqueOff() {
    digitalWrite(this->PWM1, LOW);
    digitalWrite(this->PWM2, LOW);
    digitalWrite(this->EN, LOW);
}

void Servo::torqueOn() {
    digitalWrite(this->PWM1, LOW);
    digitalWrite(this->PWM2, LOW);
    digitalWrite(this->EN, HIGH);
}
