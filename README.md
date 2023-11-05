# README

## Test bed controller

This piece runs on 1 arduino that is connected to all 3 DC motors & 3 position sensors (AS5600). Together, they constitute 3 servo motors that control the leg's position. It'll log the loop frequency and can be used to test the maximum capacity of the robot's legs. The robot is placed upside down and it's foot can be weighted to simulate landings or jumping.

<img src="https://github.com/sam-n-johnston/hopper-arduino/assets/17952091/8b82ed3b-52f0-4972-9031-ef05a9301a39" width="500" >

### Connections

For the test bed, only the servos are connected:

| Connections                        | Servo 1 | Servo 2 | Servo 3 |
|------------------------------------|---------|---------|---------|
| PWM1                               | 11      | 9       | 5       |
| PWM2                               | 10      | 6       | 3       |
| OCM                                | A0      | A1      | A2      |
| DIAG                               | 8       | 4       | A3      |
| EN                                 | 7       | 2       | 12      |
| AS5600 chip select pin on TCA9548A | 2       | 4       | 6       |
