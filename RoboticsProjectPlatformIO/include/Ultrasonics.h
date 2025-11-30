#ifndef ULTRASONICS_H
#define ULTRASONICS_H

#include <mbed.h>

class Ultrasonics {
    private:
        mbed::DigitalInOut US_1;
        mbed::DigitalInOut US_2;
        mbed::DigitalInOut US_3;
        mbed::DigitalInOut US_4;

        mbed::DigitalInOut* sensors[4];

        mbed::Timer timer;
    public:
        Ultrasonics();
        float distances[4];
        void runtime(int val);
        float correction;

        void setup();
};

#endif