#include <Ultrasonics.h>

Ultrasonics::Ultrasonics() :
US_1(P0_23),
US_2(P1_14),
US_3(P1_13),
US_4(P1_15)
{
    sensors[0] = &US_1;
    sensors[1] = &US_2;
    sensors[2] = &US_3;
    sensors[3] = &US_4;
}

void Ultrasonics::runtime(int val) {
    sensors[val]->output();
    *sensors[val] = 0;  // ensure pin is low
    *sensors[val] = 1;
    timer.reset();
    wait_us(10);        // wait 10us
    *sensors[val] = 0;  // set low

    sensors[val]->input();
    while (*sensors[val] == 0) {  };
    timer.start();
    while (*sensors[val] == 1) {  };
    timer.stop();

    distances[val] = (timer.read_us() - correction) / 58.0;  // formula from datasheet
}

void Ultrasonics::setup() {
    timer.reset();
    timer.start();
    timer.stop();
    correction = timer.read_us();
}