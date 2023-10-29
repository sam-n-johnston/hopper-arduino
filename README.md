# README

## Test bed controller

This piece runs on 1 arduino that is connected to all 3 DC motors & 3 position sensors (AS5600). Together, they constitute 3 servo motors that control the leg's position. It'll log the loop frequency and can be used to test the maximum capacity of the robot's legs. The robot is placed upside down and it's foot can be weighted to simulate landings or jumping.

<img src="[https://your-image-url.type](https://github.com/sam-n-johnston/hopper-arduino/assets/17952091/71fa3a4c-1425-4bef-8040-db1f5a0d1ad2)" width="100" height="100">


### Connections

For the test bed, only the servos are connected:

| Connections | PWM1 pin | PWM2 pin | AS5600 chip select pin on TCA9548A |
|-------------|----------|----------|------------------------------------|
| Servo 1     | 11       | 10       | 2                                  |
| Servo 2     | 6        | 9        | 6                                  |
| Servo 3     | 3        | 5        | 4                                  |




