#include <Sensors.h>

#include <Ultrasonics.h>
#include <Infrareds.h>

Sensors::Sensors(Ultrasonics &us, Infrareds &ir) 
    : US(us),
      IR(ir)
{
    // distanceRight = 0;
    // distanceFront = 0;
    // RightDistances rightIRs;
}

void Sensors::update() {
    US.runtime(0);
    US.runtime(1);

    IR.runtime();

    distanceFront = US.distances[1];
    distanceRight = US.distances[0];
    distanceArray = IR.getDistances();

    rightIRs.front = distanceArray[0];
    rightIRs.rear = distanceArray[1];
}

// float Sensors::getLeftDist() {
//     return distanceLeft_IR;
// }

RightDistances Sensors::getRightDist_IR() {  
    // Serial.println((String)"In sensors class rightIRs is: " + rightIRs.front + " " + rightIRs.rear);
    return rightIRs;
}

float Sensors::getRightDist() {
    return distanceRight;
}

float Sensors::getFrontDist() {
    return distanceFront;
}
