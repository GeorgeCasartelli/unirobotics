
// Gyro.cpp
#include <Gyro.h>

bool Gyro::begin() {
  if (!IMU.begin()) {
    return false;
  }
  angle = 0.0f;
  lastTimeUs = micros();
  return true;
}

void Gyro::update() {
  // Serial.println("GYRO CALLED");
  float x, y, z;
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x, y, z); // deg/s
    
    uint32_t now = micros();
    float dt = (now - lastTimeUs) * 1e-6f;  // convert to seconds
    lastTimeUs = now;
    
    // if (dt <= 0.0f || dt > 0.1f) return;
    angle += z * dt;  // integrate angular velocity
    // Serial.println(angle);
  }
}

void Gyro::reset() {
  angle = 0.0f;
}

float Gyro::getAngle() {
  return angle * (3.14159f / 180.0f);
}