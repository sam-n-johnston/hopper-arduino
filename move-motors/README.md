## Objective

The objective is to move a a motor using the pololu [TB9051FTG Single Brushed DC Motor Driver Carrier](https://www.pololu.com/product/2997/resources). 

## Guide

Modify the values based on where the motor is plugged in: 

```
const int PULLER_PWM1 = 1, PULLER_PWM2 = 3, PULLER_DIAG = 5,
          PULLER_EN = 7;
```

And check the it turns the motor. 

