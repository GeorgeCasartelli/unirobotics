#ifndef GYRO_H
#define GYRO_H

#include <Arduino.h>
#include <Arduino_BMI270_BMM150.h>


class Gyro {
    private:
        float angle; // degrees
        uint32_t lastTimeUs;

    public:
        bool begin();
        void reset();
        void update();
        float getAngle();
        float getAngleDeg() const { return angle; };
};

#endif