# README

## tri-servo controller

This piece runs on 1 arduino that is connected to all 3 DC motors & 3 position sensors (AS5600). Together, they constitute 3 servo motors. It'll log the loop frequency and can be used to test the maximum capacity of the robot's legs. 

## Connections

For the test bed, only the servos are connected:

| Connections | PWM1 pin | PWM2 pin | AS5600 chip select pin on TCA9548A |
|-------------|----------|----------|------------------------------------|
| Servo 1     | 11       | 10       | 2                                  |
| Servo 2     | 6        | 9        | 6                                  |
| Servo 3     | 3        | 5        | 4                                  |




