#ifndef INFRAREDS_H
#define INFRAREDS_H

#include <mbed.h>

class Infrareds{
    

    public:
        Infrareds();
        void runtime();

        float distances[4];

    private:

        mbed::I2C i2c;
        const char I2C_ADDR = 0x80;
        const char DISTANCE_ADDR = 0x5E;

        const char MUX_CMD[4] = { 0x01, 0x02, 0x04, 0x08 };
        const char MUX_ADDR = 0xEE;

        char data[2];
};



#endif