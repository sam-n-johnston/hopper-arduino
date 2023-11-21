#include "spiServo.h"

SPIServo::SPIServo(
    uint8_t chipSelectPin,
    uint8_t getPositionQuery,
    uint8_t setPositionCommand,
    uint8_t torqueOffCommand,
    uint8_t torqueOnCommand) {
    this->chipSelectPin = chipSelectPin;
    this->getPositionQuery = getPositionQuery;
    this->setPositionCommand = setPositionCommand;
    this->torqueOffCommand = torqueOffCommand;
    this->torqueOnCommand = torqueOnCommand;
}

void SPIServo::begin() {
    pinMode(this->chipSelectPin, OUTPUT);
    digitalWrite(this->chipSelectPin, HIGH);

    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV4);
    Serial.println("SPI Servo Setup Done");
};

float SPIServo::getCurrentPosition() {
    digitalWrite(this->chipSelectPin, LOW);

    delayMicroseconds(50);
    byte m_receive1 = SPI.transfer(this->getPositionQuery);
    delayMicroseconds(50);
    byte m_receive2 = SPI.transfer(0xFF);
    delayMicroseconds(50);
    byte m_receive3 = SPI.transfer(0xFF);
    delayMicroseconds(50);

    digitalWrite(this->chipSelectPin, HIGH);

    int result = m_receive2 << 8 | m_receive3;

    return result;
}

void SPIServo::setPositionInDeg(float desiredPosition) {
    if (desiredPosition > 30.0 || desiredPosition < -90.0) {
        Serial.println("Tried to set position outside acceptable range");
        desiredPosition = 0.0;
    }

    uint8_t storage[5];
    storage[0] = this->setPositionCommand;
    memcpy(storage + 1, &desiredPosition, 4);

    digitalWrite(this->chipSelectPin, LOW);

    delayMicroseconds(50);
    SPI.transfer(storage[0]);
    delayMicroseconds(50);
    SPI.transfer(storage[1]);
    delayMicroseconds(50);
    SPI.transfer(storage[2]);
    delayMicroseconds(50);
    SPI.transfer(storage[3]);
    delayMicroseconds(50);
    SPI.transfer(storage[4]);
    delayMicroseconds(50);

    digitalWrite(this->chipSelectPin, HIGH);
};

void SPIServo::torqueOn() {
    digitalWrite(this->chipSelectPin, LOW);

    delay(15);
    byte m_receive1 = SPI.transfer(this->torqueOnCommand);
    delay(15);

    digitalWrite(this->chipSelectPin, HIGH);
}

void SPIServo::torqueOff() {
    digitalWrite(this->chipSelectPin, LOW);

    delay(15);
    byte m_receive1 = SPI.transfer(this->torqueOffCommand);
    delay(15);

    digitalWrite(this->chipSelectPin, HIGH);
}
