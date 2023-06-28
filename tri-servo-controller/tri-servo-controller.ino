#include "Servo.h"

Servo servo1 = Servo(11, 10, 4, 611, true);
Servo servo2 = Servo(6, 9, 2, 2315, false);
Servo servo3 = Servo(3, 5, 6, 373, true);

void setup() {
  Serial.begin(115200);
  servo1.begin();
  servo2.begin();
  servo3.begin();
}

void loop() {
  int pos = servo2.getCurrentPosition();
  Serial.println(pos);
  long int time = millis() / 1000;
  if (time % 2 == 0) {
    servo1.setPositionInDeg(0);
    servo2.setPositionInDeg(0);
    servo3.setPositionInDeg(0);
  } else {
    servo1.setPositionInDeg(100);
    servo2.setPositionInDeg(100);
    servo3.setPositionInDeg(100);
  }
}
