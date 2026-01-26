#include <Sensors.h>

#include <Ultrasonics.h>
#include <Infrareds.h>
#include <Gyro.h>


Sensors::Sensors(Ultrasonics &us, Infrareds &ir, Gyro &gyro) 
    : ultrasonics(us),
      infrareds(ir),
      GYRO(gyro)
{

}
void Sensors::setup() {
    ultrasonics.setup();
    GYRO.begin();
}

void Sensors::update() {
    GYRO.update();

    ultrasonics.runtime(0); // right
    ultrasonics.runtime(1); // front
    ultrasonics.runtime(2); // front left
    ultrasonics.runtime(3); // front right

    infrareds.runtime();

    // read ultrasonic distances
    distanceFront = ultrasonics.distances[1];
    distanceRight = ultrasonics.distances[0];
    distanceFrontLeft = ultrasonics.distances[2];
    distanceFrontRight = ultrasonics.distances[3];
   
    // read IR distances
    float* irDistances = infrareds.getDistances();
    irArray.right = irDistances[0];
    irArray.left = irDistances[1];

    addIRReading(irArray.right);
}


// filtering
void Sensors::addIRReading(float reading) {    
    // add to circular buffer
    irBuffer[bufferIndex] = reading;
    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
    
    // mark buffer as full once we've wrapped around
    if (bufferIndex == 0) {
        bufferFull = true;
    }
}

float Sensors::getRightIRFiltered() const {
    // return raw if buffer not full
    if (!bufferFull) {
        return irArray.right;
    }
    
    // get avg
    float sum = 0.0f;
    for (int i = 0; i < BUFFER_SIZE; i++) {
        sum += irBuffer[i];
    }
    
    return sum / BUFFER_SIZE;
}