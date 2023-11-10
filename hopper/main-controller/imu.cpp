#include "imu.h"

IMU::IMU() {}

void IMU::begin()
{
  DEBUG_SERIAL.print("Starting IMU... ");
  this->bno = Adafruit_BNO055(55, 0x28, &Wire);

  if (!this->bno.begin())
  {
    DEBUG_SERIAL.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }

  DEBUG_SERIAL.println("DONE");

  // Changes the orientation, but not as expected based on the docs
  // this->bno.setAxisRemap(0x24);
}

Vector IMU::getLinearAcceleration()
{
  sensors_event_t linearAccelData, gravityData;
  this->bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  Vector linearAcceleration;
  linearAcceleration.x = linearAccelData.acceleration.x;
  linearAcceleration.y = linearAccelData.acceleration.y;
  linearAcceleration.z = linearAccelData.acceleration.z;
  unsigned long timeSinceLastMeasurementInMs = millis() - this->lastAccelerationMeasurementTimeInMs;

  this->lastComputedSpeed.x = linearAcceleration.x * timeSinceLastMeasurementInMs;
  this->lastComputedSpeed.y = linearAcceleration.y * timeSinceLastMeasurementInMs;
  this->lastComputedSpeed.z = linearAcceleration.z * timeSinceLastMeasurementInMs;

  this->lastAccelerationMeasurementTimeInMs = millis();

  // DEBUG_SERIAL.print("Got linear accelerations - x: ");
  // DEBUG_SERIAL.print(linearAcceleration.x);
  // DEBUG_SERIAL.print(", y: ");
  // DEBUG_SERIAL.print(linearAcceleration.y);
  // DEBUG_SERIAL.print(", z: ");
  // DEBUG_SERIAL.print(linearAcceleration.z);
  // DEBUG_SERIAL.println();

  return linearAcceleration;
}

Vector IMU::getGravity()
{
  sensors_event_t gravityData;
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  Vector gravity;
  gravity.x = -gravityData.acceleration.x;
  gravity.y = -gravityData.acceleration.y;
  gravity.z = -gravityData.acceleration.z;

  // DEBUG_SERIAL.print("Got gravity - x: ");
  // DEBUG_SERIAL.print(gravity.x);
  // DEBUG_SERIAL.print(", y: ");
  // DEBUG_SERIAL.print(gravity.y);
  // DEBUG_SERIAL.print(", z: ");
  // DEBUG_SERIAL.print(gravity.z);
  // DEBUG_SERIAL.println();

  return gravity;
}

Vector IMU::getOrientation()
{
  sensors_event_t orientationData;
  this->bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  Vector orientation;
  orientation.x = -orientationData.orientation.z;
  orientation.y = -orientationData.orientation.y;
  orientation.z = -orientationData.orientation.x;

  // DEBUG_SERIAL.print("Got orientation - x: ");
  // DEBUG_SERIAL.print(orientation.x);
  // DEBUG_SERIAL.print(", y: ");
  // DEBUG_SERIAL.print(orientation.y);
  // DEBUG_SERIAL.print(", z: ");
  // DEBUG_SERIAL.print(orientation.z);
  // DEBUG_SERIAL.println();

  return orientation;
}

Vector IMU::getComputedLinearVelocity()
{
  return this->lastComputedSpeed;
}

Vector IMU::getAngularVelocity()
{
  sensors_event_t angVelocityData;
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);

  Vector angularVelocity;
  angularVelocity.x = -angVelocityData.gyro.z;
  angularVelocity.y = angVelocityData.gyro.y;
  angularVelocity.z = -angVelocityData.gyro.x;

  // DEBUG_SERIAL.print("Got angularVelocity - x: ");
  // DEBUG_SERIAL.print(angularVelocity.x);
  // DEBUG_SERIAL.print(", y: ");
  // DEBUG_SERIAL.print(angularVelocity.y);
  // DEBUG_SERIAL.print(", z: ");
  // DEBUG_SERIAL.print(angularVelocity.z);
  // DEBUG_SERIAL.println();

  return angularVelocity;
}