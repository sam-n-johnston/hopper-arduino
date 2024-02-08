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

const int SER1_PWM1 = 9, SER1_PWM2 = 6, SER1_OCM = A1, SER1_DIAG = 4,
          SER1_EN = 2, SER1_AS5600 = 4;
const int SER3_PWM1 = 5, SER3_PWM2 = 3, SER3_OCM = A2, SER3_DIAG = A3,
          SER3_EN = A0, SER3_AS5600 = 6;

Servo servo1 = Servo(
    SER1_PWM1,
    SER1_PWM2,
    SER1_OCM,
    SER1_DIAG,
    SER1_EN,
    SER1_AS5600,
    105,
    true);
Servo servo3 = Servo(
    SER3_PWM1,
    SER3_PWM2,
    SER3_OCM,
    SER3_DIAG,
    SER3_EN,
    SER3_AS5600,
    841,
    true);

#define COMMAND_SET_GOAL_POSITION1 0
#define COMMAND_SET_GOAL_POSITION3 1
#define QUERY_GET_POSITION1 2
#define QUERY_GET_POSITION3 3
#define TORQUE_OFF1 4
#define TORQUE_OFF3 5
#define TORQUE_ON1 6
#define TORQUE_ON3 7

// Change to int when testing 2 bytes.
byte goalPosition = 0;
bool gettingCurrentPosition = false;
int chipSelectPin = 10;

// Used for SPI communication
byte bytes[5];
volatile byte pos;
volatile boolean readyToProcessData;

volatile byte replyPosition;
volatile int positionToSend;
volatile boolean sendingPosition;
volatile byte request;
int currentPositionThatIsBeingSent;
float currentGoalPosition1 = 0.0;
float currentGoalPosition3 = 0.0;
bool torqueOn1 = false;
bool torqueOn3 = false;
volatile bool posted = false;

void setup() {
    Serial.begin(115200);
    servo1.begin();
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

    if (pos == 0 &&
        (data == QUERY_GET_POSITION1 || data == QUERY_GET_POSITION3)) {
        // Set current position in memory
        if (data == QUERY_GET_POSITION1)
            currentPositionThatIsBeingSent = servo1.getMostRecentPosition();
        else
            currentPositionThatIsBeingSent = servo3.getMostRecentPosition();

        byte firstByte = currentPositionThatIsBeingSent >> 8;
        SPDR = firstByte;
        replyPosition++;
        sendingPosition = true;
        return;
    }

    if (pos == 0 && data == TORQUE_OFF1) {
        torqueOn1 = false;
        return;
    }

    if (pos == 0 && data == TORQUE_OFF3) {
        torqueOn3 = false;
        return;
    }

    if (pos == 0 && data == TORQUE_ON1) {
        torqueOn1 = true;
        return;
    }

    if (pos == 0 && data == TORQUE_ON3) {
        torqueOn3 = true;
        return;
    }

    bytes[pos++] = data;

    if (pos >= 5) {
        pos = 0;
        if (bytes[0] == COMMAND_SET_GOAL_POSITION1)
            memcpy(&currentGoalPosition1, bytes + 1, 4);
        else
            memcpy(&currentGoalPosition3, bytes + 1, 4);
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

    if (torqueOn1)
        servo1.setPositionInDeg(currentGoalPosition1);
    else {
        servo1.getCurrentPosition();
        servo1.torqueOff();
    }

    if (torqueOn3)
        servo3.setPositionInDeg(currentGoalPosition3);
    else {
        servo3.getCurrentPosition();
        servo3.torqueOff();
    }
}
