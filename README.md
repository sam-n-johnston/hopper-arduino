# README

## Test bed controller

This piece runs on 1 arduino that is connected to all 3 DC motors & 3 position sensors (AS5600). Together, they constitute 3 servo motors that control the leg's position. It'll log the loop frequency and can be used to test the maximum capacity of the robot's legs. The robot is placed upside down and it's foot can be weighted to simulate landings or jumping.

<img src="https://github.com/sam-n-johnston/hopper-arduino/assets/17952091/8b82ed3b-52f0-4972-9031-ef05a9301a39" width="500" >

### Connections

For the test bed, only the servos are connected:

| Connections                        | Servo 1 | Servo 2 | Servo 3 |
|------------------------------------|---------|---------|---------|
| PWM1                               | 3 (main)      | 9       | 5       |
| PWM2                               | 5 (main)      | 6       | 3       |
| OCM                                | A0 (main)      | A1      | A2      |
| DIAG                               | 7 (main)       | 4       | A3      |
| EN                                 | 8 (main)       | 2       | A0      |
| AS5600 chip select pin on TCA9548A | -       | 4       | 6       |


## Full Robot

This piece runs on 2 Arduinos. One of them takes care of 1 servo, the IMU and the commands to the secondary Arduino, which controls 2 servos. The servos are DC motors with position sensors (AS5600). 

### Connections

For the test bed, only the servos are connected:

| Connections                        | Servo 1 | Servo 2 | Servo 3 |
|------------------------------------|---------|---------|---------|
| PWM1                               | 3 (main)      | 9       | 5       |
| PWM2                               | 5 (main)      | 6       | 3       |
| OCM                                | A0 (main)      | A1      | A2      |
| DIAG                               | 7 (main)       | 4       | A3      |
| EN                                 | 8 (main)       | 2       | A0      |
| AS5600 chip select pin on TCA9548A | -       | 4       | 6       |

On the main Arduino, pin 10 is used as chip select for the SPI communication to the servo-controller.

