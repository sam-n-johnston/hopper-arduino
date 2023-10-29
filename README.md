# README

## Test bed controller

This piece runs on 1 arduino that is connected to all 3 DC motors & 3 position sensors (AS5600). Together, they constitute 3 servo motors that control the leg's position. It'll log the loop frequency and can be used to test the maximum capacity of the robot's legs. The robot is placed upside down and it's foot can be weighted to simulate landings or jumping.

<img src="https://github.com/sam-n-johnston/hopper-arduino/assets/17952091/8b82ed3b-52f0-4972-9031-ef05a9301a39" width="500" >

### Connections

For the test bed, only the servos are connected:

| Connections | PWM1 pin | PWM2 pin | AS5600 chip select pin on TCA9548A |
|-------------|----------|----------|------------------------------------|
| Servo 1     | 11       | 10       | 2                                  |
| Servo 2     | 6        | 9        | 6                                  |
| Servo 3     | 3        | 5        | 4                                  |




