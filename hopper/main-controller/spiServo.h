#ifndef SPI_SERVO_H
#define SPI_SERVO_H

#include "AS5600.h"
#include "Wire.h"
#include "servo.h"
#include <stdint.h>

#define COMMAND_SET_GOAL_POSITION2 0
#define COMMAND_SET_GOAL_POSITION3 1
#define QUERY_GET_POSITION2 2
#define QUERY_GET_POSITION3 3
#define TORQUE_OFF2 4
#define TORQUE_OFF3 5
#define TORQUE_ON2 6
#define TORQUE_ON3 7

class SPIServo : public IServo {
private:
    // This is the chip select pin for the Arduino that controls this servo
    uint8_t chipSelectPin;
    // The queries/commands are set based on the number of the servo
    uint8_t getPositionQuery;
    uint8_t setPositionCommand;
    uint8_t torqueOffCommand;
    uint8_t torqueOnCommand;

    void setMotorTorque(float speed);

public:
    SPIServo(
        uint8_t chipSelectPin,
        uint8_t getPositionQuery,
        uint8_t setPositionCommand,
        uint8_t torqueOffCommand,
        uint8_t torqueOnCommand);

    void begin();
    int getCurrentPosition();
    void setPositionInDeg(float deg);
    void torqueOn();
    void torqueOff();
};

#endif