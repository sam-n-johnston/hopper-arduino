#include "Servo.h"
#include "InverseKinematics.h"

Servo servo1 = Servo(11, 10, 4, 611, true);
Servo servo2 = Servo(6, 9, 2, 2315, false);
Servo servo3 = Servo(3, 5, 6, 373, true);

void setup() {
  Serial.begin(115200);
  servo1.begin();
  servo2.begin();
  servo3.begin();
}

int lastSecond = 0;
int loops = 0;

void loop() {
  loops++;
  long int currTime = millis();

  if (lastSecond + 1000 < currTime) {
    lastSecond = currTime;
    Serial.print("Current hz: ");
    Serial.println(loops);
    loops = 0;
  }

  long int time = millis();
  float theta1;
  float theta2;
  float theta3;

  int timeInterval = 2000;

  if (time < timeInterval) {
    int status = delta_calcInverse(0, 0, -100, theta1, theta2, theta3);

    servo1.setPositionInDeg(theta1);
    servo2.setPositionInDeg(theta2);
    servo3.setPositionInDeg(theta3);
  } else if (time < 2 * timeInterval) {
    int status = delta_calcInverse(0, 0, -150, theta1, theta2, theta3);

    servo1.setPositionInDeg(theta1);
    servo2.setPositionInDeg(theta2);
    servo3.setPositionInDeg(theta3);
  } else if (time < 3 * timeInterval) {
    int status = delta_calcInverse(50, 0, -100, theta1, theta2, theta3);

    servo1.setPositionInDeg(theta1);
    servo2.setPositionInDeg(theta2);
    servo3.setPositionInDeg(theta3);
  } else if (time < 4 * timeInterval) {
    int status = delta_calcInverse(-50, 0, -100, theta1, theta2, theta3);

    servo1.setPositionInDeg(theta1);
    servo2.setPositionInDeg(theta2);
    servo3.setPositionInDeg(theta3);
  } else if (time < 5 * timeInterval) {
    int status = delta_calcInverse(0, 50, -100, theta1, theta2, theta3);

    servo1.setPositionInDeg(theta1);
    servo2.setPositionInDeg(theta2);
    servo3.setPositionInDeg(theta3);
  } else if (time < 6 * timeInterval) {
    int status = delta_calcInverse(0, -50, -100, theta1, theta2, theta3);

    servo1.setPositionInDeg(theta1);
    servo2.setPositionInDeg(theta2);
    servo3.setPositionInDeg(theta3);
  } else {
    int status = delta_calcInverse(0, 0, -100, theta1, theta2, theta3);

    servo1.setPositionInDeg(theta1);
    servo2.setPositionInDeg(theta2);
    servo3.setPositionInDeg(theta3);
  }
}

  // Serial.print(status);
  // Serial.print(": ");
  // Serial.print(theta1);
  // Serial.print(" - ");
  // Serial.print(theta2);
  // Serial.print(" - ");
  // Serial.print(theta3);
  // Serial.println();