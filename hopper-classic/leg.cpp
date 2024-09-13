#include "leg.h"

Leg::Leg(Puller *puller, int servoXId, int servoYId) {
    this->puller = puller;
    this->servoXId = servoXId;
    this->servoYId = servoYId;
}

void Leg::begin() {
    this->puller->begin();

    const int DXL_DIR_PIN = 28; // DYNAMIXEL Shield DIR PIN - Dummy pin here
    const float DXL_PROTOCOL_VERSION = 2.0;

    this->dxl = Dynamixel2Arduino(Serial2, DXL_DIR_PIN);
    dxl.begin(57600);
    delay(50);
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
    bool pingResX = dxl.ping(servoXId);
    bool pingResY = dxl.ping(servoYId);
    Serial.print("Ping Servo X: ");
    Serial.print(pingResX);
    Serial.print(" - Y: ");
    Serial.println(pingResY);

    dxl.torqueOff(servoXId);
    delay(50);
    dxl.setOperatingMode(servoXId, OP_POSITION);
    delay(50);
    dxl.torqueOn(servoXId);

    delay(50);
    dxl.torqueOff(servoYId);
    delay(50);
    dxl.setOperatingMode(servoYId, OP_POSITION);
    delay(50);
    dxl.torqueOn(servoYId);

    Serial.println("Leg Setup complete");
}

/**
 * @brief 
 * 
 * @return float: returns the angle of the leg in relation to the body
 */
float Leg::getAlphaXInDeg() {
    int present_position = dxl.getPresentPosition(servoXId);
    return present_position;
}

/**
 * @brief 
 * 
 * @return float: returns the angle of the leg in relation to the body
 */
float Leg::getAlphaYInDeg() {
    int present_position = dxl.getPresentPosition(servoYId);
    return present_position;
}

void Leg::setDesiredAlphaXYInDeg(float degX, float degY) {
    float limit = 35.0;
    float xOffset = 0.0;
    float yOffset = 0.0;
    float desiredDegX = degX + xOffset;
    float desiredDegY = degY + yOffset;

    if (desiredDegX > limit)
        desiredDegX = limit;
    if (desiredDegX < -limit)
        desiredDegX = -limit;
    if (desiredDegY > limit)
        desiredDegY = limit;
    if (desiredDegY < -limit)
        desiredDegY = -limit;

    float limitedPositionX = 180.0 + desiredDegX;
    float limitedPositionY = 180.0 - desiredDegY;

    int finalRawGoalPositionX = limitedPositionX / 360.0 * 4096;
    int finalRawGoalPositionY = limitedPositionY / 360.0 * 4096;

    // Serial.print("finalRawGoalPositionY: ");
    // Serial.println(finalRawGoalPositionY);

    dxl.setGoalPosition(servoXId, finalRawGoalPositionX);
    delay(5);
    dxl.setGoalPosition(servoYId, finalRawGoalPositionY);
}

void Leg::pushDown() {
    this->puller->setPositionInDeg(-85.0);
}

void Leg::stopPushingDown() {
    this->puller->setPositionInDeg(0.0);
}

bool Leg::isFootTouchingGround() {
    int sensorValue = analogRead(this->footSensorPin);

    // Serial.print("Sensor value: ");
    // Serial.println(sensorValue);

    if (sensorValue > 50)
        return true;

    return false;
}

void Leg::torqueOff() {
    this->puller->torqueOff();
    dxl.torqueOff(servoXId);
    dxl.torqueOff(servoYId);
}

void Leg::torqueOn() {
    // this->puller->torqueOn();
    dxl.torqueOn(servoXId);
    dxl.torqueOn(servoYId);
}
