#ifndef CONTROLLERS_H
#define CONTROLLERS_H

#include <mbed.h>
#include <Arduino.h>
#include <Motors.h>

class Controllers{
    private:
        
        float distanceTarget;
        float startDistanceA;
        float startDistanceB;
        float movingForward;
        float turnTarget;
        float startAngle;
        
        float maxTurnSpeed;
        float minTurnSpeed;

        const float trackWidth = 110.0f;

        bool printStates = true;

        enum STATES {
            IDLE,
            MOVING,
            TURNING
        };

        STATES controllerState = IDLE;

        const char* stateToString(STATES s);

        float calculateTrapezoidalSpeed(float traveled, float maxSpeed, float accelDist, float decelDist, float error);
        
        Motors &motors;

    public:
        Controllers(Motors &motor);
        void moveDistance(float target, bool forward);
        void turnDegrees(float angle);
        void update();
        
};


#endif 