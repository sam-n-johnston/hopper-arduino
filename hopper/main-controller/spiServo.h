#ifndef SPI_SERVO_H
#define SPI_SERVO_H

#include "AS5600.h"
#include "servo.h"
#include <SPI.h>
#include <stdint.h>

#define COMMAND_SET_GOAL_POSITION1 0x00
#define COMMAND_SET_GOAL_POSITION3 0x01
#define QUERY_GET_POSITION1 0x02
#define QUERY_GET_POSITION3 0x03
#define TORQUE_OFF1 0x04
#define TORQUE_OFF3 0x05
#define TORQUE_ON1 0x06
#define TORQUE_ON3 0x07

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