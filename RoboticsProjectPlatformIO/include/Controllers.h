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

        bool obstacleDetected;

        const float trackWidth = 115.0f;

        bool printStates = false;

        enum STATES {
            IDLE,
            MOVING,
            TURNING,
            CONTINUOUS
        };

        STATES controllerState = IDLE;

        const char* stateToString(STATES s);

        float calculateTrapezoidalSpeed(float traveled, float totalDistance, float maxSpeed);
        
        Motors &motors;

        const float TARGET_WALL_DIST = 1500.0f;

        float rightDistanceAvg;

    public:
        Controllers(Motors &motor); 
        void moveDistance(float target, bool forward);
        void turnDegrees(float angle);
        void update();
        void setObstacleDetected(bool flag);
        void moveContinuous(bool forward, float speed = 0.7f);
        bool isIdle();
        float getAvgDistance();
        void wallFollow(float error, float baseSpeed, float Kp);
        void updateRightWall(float val);
        
        
};


#endif 