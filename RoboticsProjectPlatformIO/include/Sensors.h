#ifndef SENSORS_H
#define SENSORS_H

#include <Ultrasonics.h>
#include <Infrareds.h>
#include <Gyro.h>


struct RightDistances {
    float right;
    float left;
};

class Sensors {
public:
    Sensors(Ultrasonics &us, Infrareds &ir, Gyro &gyro);

    void setup();
    void update();   

    // dist queries
    float getFrontDist() const { return distanceFront; }  // mm
    float getFrontLeftDist() const { return distanceFrontLeft; }
    float getFrontRightDist() const { return distanceFrontRight; }
    float getRightDist() const { return distanceRight; }  // mm
    // float getLeftDist();
    RightDistances getRightDist_IR();

    float getRightAvg();

    float getRightIRFiltered() const;


private:
    // hw refs
    Ultrasonics &ultrasonics;
    Infrareds &infrareds;
    Gyro &GYRO;

    // sensor readings
    float distanceFront = 0.0f;
    float distanceRight = 0.0f;
    float distanceFrontLeft = 0.0f;
    float distanceFrontRight = 0.0f;
    
    RightDistances irArray = {0.0f, 0.0f};

    static constexpr int BUFFER_SIZE = 20;
    float irBuffer[BUFFER_SIZE] = {0};
    int bufferIndex = 0;
    bool bufferFull = false;
    
    void addIRReading(float reading);
};


#endif