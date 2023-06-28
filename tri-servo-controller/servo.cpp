#include "servo.h"
#define TCAADDR 0x70

void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

Servo::Servo(
    uint8_t clockWiseDirectionPin,
    uint8_t counterClockWiseDirectionPin,
    uint8_t as5600Pin,
    int zeroPosition,
    bool direction
) {
  this->clockWiseDirectionPin = clockWiseDirectionPin;
  this->counterClockWiseDirectionPin = counterClockWiseDirectionPin;
  this->as5600Pin = as5600Pin;
  this->zeroPosition = zeroPosition;
  this->direction = direction;
}

void Servo::begin()
{
    // Setup gear motor
    pinMode(this->clockWiseDirectionPin, OUTPUT);
    pinMode(this->counterClockWiseDirectionPin, OUTPUT);
    digitalWrite(this->clockWiseDirectionPin, LOW);
    digitalWrite(this->counterClockWiseDirectionPin, LOW);

    tcaselect(this->as5600Pin);
    // Setup encoder
    as5600.begin(4);                        //  set direction pin.
    as5600.setDirection(AS5600_CLOCK_WISE); // default, just be explicit.

    int b = as5600.isConnected();
};

int Servo::getCurrentPosition()
{
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

float Servo::getPIDOutput(float error)
{
    float kp = 3.; // Pc ~140?
    float kd = 0.0;
    float ki = 0.0;

    float deltaError = (this->previousError - error) / this->deltaTime;

    float output = kp * error + kp * deltaError + ki * this->integralError;

    return output;
}

void Servo::setPositionInDeg(float desiredPosition)
{
    this->deltaTime = millis() - previousPositionTime;
    int currentPosition = this->getCurrentPosition();

    float error = desiredPosition - currentPosition;
    this->integralError += error * this->deltaTime;

    float output = this->getPIDOutput(error);

    setMotorTorque(output);

    this->previousPositionTime = millis();
    this->previousError = error;
};

void Servo::setMotorTorque(float speed)
{
    // The level at which the motor starts moving
    float zeroSpeed = 0.12 / 5.0 * 255.0;
    float maxSpeed = 255.0;

    float adjustedSpeed = speed == 0 ? 0.0 : abs(speed) + zeroSpeed;

    if (adjustedSpeed > maxSpeed)
        adjustedSpeed = maxSpeed;


      // analogWrite(this->counterClockWiseDirectionPin, 0);
      // analogWrite(this->clockWiseDirectionPin, 0);
      if (speed > 0) {
          analogWrite(this->counterClockWiseDirectionPin, round(adjustedSpeed));
          analogWrite(this->clockWiseDirectionPin, 0);
      } else {
          analogWrite(this->counterClockWiseDirectionPin, 0);
          analogWrite(this->clockWiseDirectionPin, round(adjustedSpeed));
      }
}

void Servo::torqueOff()
{
    digitalWrite(this->counterClockWiseDirectionPin, LOW);
    digitalWrite(this->clockWiseDirectionPin, LOW);
}
