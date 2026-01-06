#ifndef SENSORS_H
#define SENSORS_H

#include <Ultrasonics.h>
#include <Infrareds.h>



struct RightDistances {
    float front;
    float rear;
};

class Sensors {
public:
    Sensors(Ultrasonics &us, Infrareds &ir);

    void update();   // call every loop

    // Raw values (smoothed)
    float getFrontDist();  // mm
    float getRightDist();  // mm
    float getLeftDist();
    RightDistances getRightDist_IR();

    float getRightAvg();
    // String getValues() {

    // }

private:
    Ultrasonics &US;
    Infrareds &IR;

    float distanceFront;
    float distanceRight;
    
    float* distanceArray;

    RightDistances rightIRs;
    
    int bufferIndex;
    bool bufferFull;

    static constexpr int BUFFER_SIZE = 20;

    float distanceBufferIR[BUFFER_SIZE];

    void addReading(float reading);
};


#endif