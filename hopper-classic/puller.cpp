#include "puller.h"
#define TCAADDR 0x70

Puller::Puller(
    uint8_t PWM1,
    uint8_t PWM2,
    uint8_t OCM,
    uint8_t DIAG,
    uint8_t EN,
    bool direction) {
    this->PWM1 = PWM1;
    this->PWM2 = PWM2;
    this->OCM = OCM;
    this->DIAG = DIAG;
    this->EN = EN;
    this->zeroPosition = zeroPosition;
    this->direction = direction;
}

void Puller::begin() {
    Serial.println("Setting up puller");

    // Setup gear motor
    pinMode(this->PWM1, OUTPUT);
    pinMode(this->PWM2, OUTPUT);
    pinMode(this->OCM, INPUT);
    pinMode(this->DIAG, INPUT);
    pinMode(this->EN, OUTPUT);
    digitalWrite(this->PWM1, LOW);
    digitalWrite(this->PWM2, LOW);

    Wire1.begin();
    this->as5600 = AS5600(&Wire1);
    
    // Setup encoder
    this->as5600.begin(4);                        //  set direction pin.
    this->as5600.setDirection(AS5600_CLOCK_WISE); // default, just be explicit.

    int b = as5600.isConnected();

    if (b == 0) {
        Serial.print("- AS5600 failed to connect! Retrying... - ");
        while (1) {
            delay(10);
        }
    }

    findZeroPosition();

    Serial.println("Done");
};

float Puller::getMotorCurrent() {
    float voltage = analogRead(this->OCM) / 4096.0 * 5; // 5 volt
    float motorCurrent = voltage / 0.5; // 500mV per amp

    return motorCurrent;
}

void Puller::findZeroPosition(){
    this->torqueOn();
    float initialPosition = this->getCurrentPosition();
    float currentPosition = initialPosition;
    Serial.print("Initial position is: ");
    Serial.print(initialPosition);

    float averageCurrentTotal = 0;
    float averageCurrent = 0;
    int numberOfMeasures = 10;
    float motorCurrent = 0.0;

    // Move back 1 turn
    while(motorCurrent < 0.75 && abs(currentPosition - initialPosition) < 90.0) {
        this->setMotorTorque(50);
        motorCurrent = this->getMotorCurrent();
        currentPosition = this->getCurrentPosition();

        Serial.print("Current1 is: ");
        Serial.print(motorCurrent);
        Serial.print("; Position: ");
        Serial.println(currentPosition);
    }

    // Reset & move forward 2 turns
    initialPosition = currentPosition;
    while(
        motorCurrent < 0.75 && 
        abs(currentPosition - initialPosition) < 360.0 &&
        (averageCurrent == 0 || motorCurrent < averageCurrent * 1.25)
    ) {
        this->setMotorTorque(-50);
        motorCurrent = this->getMotorCurrent();
        currentPosition = this->getCurrentPosition();

        if (numberOfMeasures > 0) {
            averageCurrentTotal += motorCurrent;
            numberOfMeasures--;
        }

        if (averageCurrent == 0 && numberOfMeasures == 0) { 
            averageCurrent = averageCurrentTotal / 10.0;
        }

        Serial.print("Current2 is: ");
        Serial.print(motorCurrent);
        Serial.print("; Position: ");
        Serial.println(currentPosition);
    }

    Serial.print("DONE: ");
    Serial.println(averageCurrent);
    this->torqueOff();
}

void Puller::goToDesiredPosition() {
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
}

float Puller::getCurrentPosition() {
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

    this->mostRecentPosition = result * 360.0 / 4096.0;

    return this->mostRecentPosition;
}

float Puller::getMostRecentPosition() {
    return mostRecentPosition;
}

float Puller::getPIDOutput(float error) {
    float kp = 2.0; // Pc ~140?
    float kd = 0.0;
    float ki = 0.0;

    float deltaError = (this->previousError - error) / this->deltaTime / 1000.0;

    float output = kp * error + kd * deltaError + ki * this->integralError;

    return output;
}

void Puller::setPositionInDeg(float desiredPosition) {
    this->desiredPosition = desiredPosition;
};

void Puller::setMotorTorque(float speed) {
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
        digitalWrite(this->PWM1, 0);
        analogWrite(this->PWM2, round(adjustedSpeed));
    } else {
        analogWrite(this->PWM1, round(adjustedSpeed));
        digitalWrite(this->PWM2, 0);
    }
}

void Puller::torqueOff() {
    digitalWrite(this->PWM1, LOW);
    digitalWrite(this->PWM2, LOW);
    digitalWrite(this->EN, LOW);
}

void Puller::torqueOn() {
    digitalWrite(this->PWM1, LOW);
    digitalWrite(this->PWM2, LOW);
    digitalWrite(this->EN, HIGH);
}
