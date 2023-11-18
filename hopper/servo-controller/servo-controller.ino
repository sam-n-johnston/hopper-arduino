/*******************************************************************************
 * Copyright 2016 ROBOTIS CO., LTD.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/

// This namespace is required to use Control table item names
//  using namespace ControlTableItem;

#include "Servo.h"
#include "Wire.h"
#include <SPI.h>

const int SER2_PWM1 = 9, SER2_PWM2 = 6, SER2_OCM = A1, SER2_DIAG = 4,
          SER2_EN = 2, SER2_AS5600 = 4;
const int SER3_PWM1 = 5, SER3_PWM2 = 3, SER3_OCM = A2, SER3_DIAG = A3,
          SER3_EN = A0, SER3_AS5600 = 6;

Servo servo2 = Servo(
    SER2_PWM1,
    SER2_PWM2,
    SER2_OCM,
    SER2_DIAG,
    SER2_EN,
    SER2_AS5600,
    3480,
    true);
Servo servo3 = Servo(
    SER3_PWM1,
    SER3_PWM2,
    SER3_OCM,
    SER3_DIAG,
    SER3_EN,
    SER3_AS5600,
    2220,
    true);

#define COMMAND_SET_GOAL_POSITION2 0
#define COMMAND_SET_GOAL_POSITION3 1
#define QUERY_GET_POSITION2 2
#define QUERY_GET_POSITION3 3
#define TORQUE_OFF2 4
#define TORQUE_OFF3 5
#define TORQUE_ON2 6
#define TORQUE_ON3 7

// Change to int when testing 2 bytes.
byte goalPosition = 0;
bool gettingCurrentPosition = false;
int chipSelectPin = 10;

// Used for SPI communication
byte bytes[3];
volatile byte pos;
volatile boolean readyToProcessData;

volatile byte replyPosition;
volatile int positionToSend;
volatile boolean sendingPosition;
volatile byte request;
int currentPositionThatIsBeingSent;
int currentGoalPosition2 = 0;
int currentGoalPosition3 = 0;
bool torqueOn2 = false;
bool torqueOn3 = false;

void setup() {
    Serial.begin(115200);
    servo2.begin();
    servo3.begin();
    pos = 0;
    readyToProcessData = false;
    replyPosition = 0;
    sendingPosition = false;

    // TODO: MISO should be floating when CS is high
    // Sets MISO as OUTPUT (Have to Send data to Master IN
    pinMode(MISO, OUTPUT);
    pinMode(chipSelectPin, INPUT);
    SPCR |= _BV(SPE); // Turn on SPI in Slave Mode

    SPI.attachInterrupt(); // Interrupt ON is set for SPI communication
    Serial.print("Done setup!");
}

// TODO: When CS goes high, clear out all information of the GET.
ISR(SPI_STC_vect) // Interrupt routine function
{
    byte data = SPDR; // grab byte from SPI Data Register

    // Transmission done, cleaning up
    if (replyPosition == 2) {
        replyPosition = 0;
        sendingPosition = false;
        return;
    }

    if (replyPosition == 1) {
        byte secondByte = currentPositionThatIsBeingSent;
        SPDR = secondByte;
        replyPosition++;
        return;
    }

    if (data == QUERY_GET_POSITION2 || data == QUERY_GET_POSITION3) {
        // Set current position in memory
        if (data == QUERY_GET_POSITION2)
            currentPositionThatIsBeingSent = servo2.getMostRecentPosition();
        else
            currentPositionThatIsBeingSent = servo3.getMostRecentPosition();

        byte firstByte = currentPositionThatIsBeingSent >> 8;
        SPDR = firstByte;
        replyPosition++;
        sendingPosition = true;
        return;
    }

    if (pos == 0 && data == TORQUE_OFF2) {
        torqueOn2 = false;
        return;
    }

    if (pos == 0 && data == TORQUE_OFF3) {
        torqueOn3 = false;
        return;
    }

    if (pos == 0 && data == TORQUE_ON2) {
        torqueOn2 = true;
        return;
    }

    if (pos == 0 && data == TORQUE_ON3) {
        torqueOn3 = true;
        return;
    }

    if (pos < sizeof(bytes)) {
        if (pos == 0)
            request = data;

        bytes[pos++] = data;
    }

    if (pos >= 3) {
        pos = 0;
        if (request == COMMAND_SET_GOAL_POSITION2)
            currentGoalPosition2 = (bytes[1] << 8) | bytes[2];
        else
            currentGoalPosition3 = (bytes[1] << 8) | bytes[2];
    }
}

long lastSecond = 0;
long loops = 0;

void loop() {
    loops++;
    long currTime = millis();

    if (lastSecond + 1000 < currTime) {
        lastSecond = currTime;
        // Serial.print("Current hz: ");
        Serial.println(loops);
        loops = 0;
    }

    if (digitalRead(chipSelectPin) == HIGH && pos > 0) {
        Serial.println("Failed to process message");
        pos = 0;
    }

    // int val = servo2.getCurrentPosition();
    // Serial.print("torque3: ");
    // Serial.println(val);

    // int zValue = -30.0 - 25.0 * sin(millis() / 100.0);

    if (torqueOn2)
        servo2.setPositionInDeg(currentGoalPosition2);
    else {
        servo2.getCurrentPosition();
        servo2.torqueOff();
    }

    if (torqueOn3)
        servo3.setPositionInDeg(currentGoalPosition3);
    else {
        servo3.getCurrentPosition();
        servo3.torqueOff();
    }
}
