#include "spiServo.h"

SPIServo::SPIServo(
    uint8_t chipSelectPin,
    uint8_t getPositionQuery,
    uint8_t setPositionCommand) {
    this->chipSelectPin = chipSelectPin;
    this->getPositionQuery = getPositionQuery;
    this->setPositionCommand = setPositionCommand;
}

void SPIServo::begin() {
    DEBUG_SERIAL.println("Setting up servos...");
    pinMode(this->chipSelectPin, OUTPUT);
    digitalWrite(this->chipSelectPin, HIGH);

    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV4);
};

int SPIServo::getCurrentPosition() {
    digitalWrite(this->chipSelectPin, LOW);

    delayMicroseconds(75);
    byte m_receive1 = SPI.transfer(this->getPositionQuery);
    delayMicroseconds(75);
    byte m_receive2 = SPI.transfer(0xFF);
    delayMicroseconds(75);
    byte m_receive3 = SPI.transfer(0xFF);
    delayMicroseconds(75);

    digitalWrite(this->chipSelectPin, HIGH);

    int result = m_receive2 << 8 | m_receive3;

    return result;
}

void SPIServo::setPositionInDeg(float desiredPosition) {
    int data = (int)desiredPosition;
    // Protect robot
    if (data > -5 || data < -130)
        data = -30;

    digitalWrite(this->chipSelectPIn, LOW);

    byte m_receive1 = SPI.transfer(this->setPositionCommand);

    byte firstByte = data >> 8;
    byte secondByte = data;

    delayMicroseconds(75);
    byte m_receive2 = SPI.transfer(firstByte);
    delayMicroseconds(75);
    byte m_receive3 = SPI.transfer(secondByte);
    delayMicroseconds(75);

    digitalWrite(this->chipSelectPIn, HIGH);
};

void SPIServo::torqueOn() {
    digitalWrite(this->chipSelectPin, LOW);

    delay(10);
    byte m_receive1 = SPI.transfer(this->torqueOnCommand);
    delay(10);

    digitalWrite(this->chipSelectPin, HIGH);
}

void SPIServo::torqueOff() {
    digitalWrite(this->chipSelectPin, LOW);

    delay(10);
    byte m_receive1 = SPI.transfer(this->torqueOffCommand);
    delay(10);

    digitalWrite(this->chipSelectPin, HIGH);
}
