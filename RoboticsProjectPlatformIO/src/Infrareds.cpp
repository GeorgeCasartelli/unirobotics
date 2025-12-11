#include <Infrareds.h>


Infrareds::Infrareds(): i2c(P0_31, P0_2) {};

void Infrareds::runtime() {
    // for all sensors
    for (int i = 0; i < 4; i++) {

        i2c.write(MUX_ADDR, &MUX_CMD[i], 1);
        i2c.write(I2C_ADDR, &DISTANCE_ADDR, 1);  // write 1st byte of cmd to addr 0x80
        wait_us(70);                             // wait x us

        data[0] = 0;
        data[1] = 0;

        int result = i2c.read(I2C_ADDR, data, 2);  // read 2 bytes

        // since i2c returns 0 on success (ack)
        if (result != 0) {
            distances[i] = 0.0f;
            continue;
        }

        // create 12 bit distance variable (bit shifted 8 bit + 4 bit)
        float distance = ((((data[0] << 4) | (data[1])) >> 1) * 0x35) / 16.0f;

        distances[i] = distance;
    }
}

float* Infrareds::getDistances() {
    return distances;
}


