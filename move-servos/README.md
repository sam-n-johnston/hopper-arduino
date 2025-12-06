## Objective

The objective is to move a XL-430 Robotis servo to make sure it's working as expected, or to move it in preparation before screwing it in

## Guide

To get this working:

1. Install the `Arduino Mbed OS RP2040 Boards` in the board manager
2. Power on robot with the external power (12V). If that's not done, then the pico cannot connect to the IMU and will not properly do it initialization. 
3. Update the `SERVO_ID` as required. 
