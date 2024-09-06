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

#include <Dynamixel2Arduino.h>


#define DXL_SERIAL   Serial2
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = 28; // DYNAMIXEL Shield DIR PIN

const uint8_t DXL_ID1 = 1;
const uint8_t DXL_ID2 = 2;
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  delay(5000);
  // put your setup code here, to run once:
  
  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);
  while(!DEBUG_SERIAL);
  DEBUG_SERIAL.print("Starting Setup...");

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  delay(50);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  bool pingRes1 = dxl.ping(DXL_ID1);
  bool pingRes2 = dxl.ping(DXL_ID2);
  DEBUG_SERIAL.print("Pings: ");
  DEBUG_SERIAL.print(pingRes1);
  DEBUG_SERIAL.print(" - ");
  DEBUG_SERIAL.println(pingRes2);

  // // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(DXL_ID1);
  delay(50);
  dxl.setOperatingMode(DXL_ID1, OP_POSITION);
  delay(50);
  dxl.torqueOn(DXL_ID1);
  DEBUG_SERIAL.println("Starting Setup2...");
  delay(50);
  dxl.torqueOff(DXL_ID2);
  delay(50);
  dxl.setOperatingMode(DXL_ID2, OP_POSITION);
  delay(50);
  dxl.torqueOn(DXL_ID2);
  DEBUG_SERIAL.println("Starting Setup3...");

  // Limit the maximum velocity in Position Control Mode. Use 0 for Max speed
  // dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, 30);
  DEBUG_SERIAL.println("Setup done!");
}

void loop() {
  // put your main code here, to run repeatedly:
  
  // Please refer to e-Manual(http://emanual.robotis.com/docs/en/parts/interface/dynamixel_shield/) for available range of value. 
  // Set Goal Position in RAW value
  // dxl.setGoalPosition(DXL_ID, 1000);
  dxl.setGoalPosition(DXL_ID1, 180, UNIT_DEGREE);
  delay(50);
  dxl.setGoalPosition(DXL_ID2, 180, UNIT_DEGREE);

  int i_present_position = 0;
  float f_present_position = 0.0;

  // while (abs(1000 - i_present_position) > 10)
  // {
  //   i_present_position = dxl.getPresentPosition(DXL_ID1);
  //   DEBUG_SERIAL.print("Present_Position(raw) : ");
  //   DEBUG_SERIAL.println(i_present_position);
  // }
  delay(1000);

  // Set Goal Position in DEGREE value
  dxl.setGoalPosition(DXL_ID1, 0, UNIT_DEGREE);
  delay(50);
  dxl.setGoalPosition(DXL_ID2, 0, UNIT_DEGREE);
  
  // while (abs(5.7 - f_present_position) > 2.0)
  // {
  //   f_present_position = dxl.getPresentPosition(DXL_ID1, UNIT_DEGREE);
  //   DEBUG_SERIAL.print("Present_Position(degree) : ");
  //   DEBUG_SERIAL.println(f_present_position);
  // }
  delay(1000);
}
