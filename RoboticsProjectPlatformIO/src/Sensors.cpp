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
    bufferIndex = 0;
    bufferFull = false;
}

void Sensors::update() {
    US.runtime(0);
    US.runtime(1);

    IR.runtime();

    distanceFront = US.distances[1];
    distanceRight = US.distances[0];
    // distanceFront = 0.0f;
    // distanceRight = 0.0f;
    distanceArray = IR.getDistances();

    rightIRs.front = distanceArray[0];
    rightIRs.rear = distanceArray[1];

    // if (distanceArray[0] < 6000) {
    //     rightIRs.front = distanceArray[0];
    // }
    // if (distanceArray[1] < 6000) {
    //     rightIRs.rear = distanceArray[1];
    // }
    

    Serial.println((String)"front: " + rightIRs.front + " rear: " + rightIRs.rear);
    addReading(rightIRs.front);
}

// float Sensors::getLeftDist() {
//     return distanceLeft_IR;
// }

RightDistances Sensors::getRightDist_IR() {  
    // Serial.println((String)"In sensors class rightIRs is: " + rightIRs.front + " " + rightIRs.rear);
    return rightIRs;
}

void Sensors::addReading(float reading) {
    if (bufferFull) {
        float lastValue = distanceBufferIR[(bufferIndex + BUFFER_SIZE - 1) % BUFFER_SIZE];
        // if (fabs(reading - lastValue) > 4000) return;
    }

    distanceBufferIR[bufferIndex] = reading;
    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE; // circular point
    if (bufferIndex == 0) bufferFull = true;
}

float Sensors::getRightAvg() {
    float sum = 0; 
    if (bufferFull) { // return avg if buffer full
        // Serial.println("Buffer full"); 
        for (int i = 0; i < BUFFER_SIZE; i++) { 
            sum+= distanceBufferIR[i]; 
        } 
        Serial.println((String)distanceBufferIR[1]); 
        return sum / BUFFER_SIZE; 
    } else { 
        return rightIRs.front; // else return reading 
    }
}

float Sensors::getRightDist() {

    return distanceRight;
}

float Sensors::getFrontDist() {
    return distanceFront;
}
