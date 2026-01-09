#ifndef CONTROLLERS_H
#define CONTROLLERS_H

#include <mbed.h>
#include <Arduino.h>
#include <Motors.h>
#include <Gyro.h>


class Controllers{
    private:
        
        float distanceTarget;
        float turnTargetDeg;
        float turnTarget;
        float startDistanceA;
        float startDistanceB;
        float movingForward;
        
        float startAngle;
        
        const float maxTurnSpeed = 0.4f;

        bool obstacleDetected;

        const float trackWidth = 112.0f;

        const float Kp = 0.01;
        const float Kp_turn = 0.8;
        const float tolerance = 0.0005;
        const float turnTolerance = 0.05;
        const float minPWM = 0.2f;

        float prevTurnError = 0.0f;

        int leftSign = 1;
        int rightSign = 1;

        // robot local pose
        float x = 0.0;
        float y = 0.0;
        float theta = 0.0;

        float prevDistA;
        float prevDistB;

        void resetPose();
        void updatePose();

        void runGoToStep();
        // STATES
        bool printStates = true;

        enum STATES {
            IDLE,
            MOVING,
            TURNING,
            GOTO
        };

        enum GoToSTATES {
            NONE,
            TURN1,
            DRIVE,
            DONE
        };

        STATES controllerState = IDLE;
        
        STATES prevState = IDLE;
        GoToSTATES gotoStep = NONE;

        float goalX = 0.0f;
        float goalY = 0.0f;
        float goalTheta = 0.0f;
        float goToHeading = 0.0f;
        float goToDistance = 0.0f;

        void transitionTo(STATES next);

        const char* stateToString(STATES s);

        float calculateTrapezoidalSpeed(float traveled, float totalDistance, float maxSpeed);
        
        Motors &motors;
        Gyro &gyro;

        const float TARGET_WALL_DIST = 1500.0f;

        float rightDistanceAvg; 

        float wrapPi(float angle) {
            while (angle > PI) angle -= 2.0f * PI;
            while (angle < PI) angle += 2.0f * PI;
            return angle;
        }

    public:
        Controllers(Motors &motor, Gyro &gyro); 
        void moveDistance(float target, bool forward);
        void turnDegrees(float angle);
        void update();
        void setObstacleDetected(bool flag);
        void moveContinuous(bool forward, float speed = 0.7f);
        bool isIdle();
        float getAvgDistance();
        void wallFollow(float error, float baseSpeed, float Kp);
        void updateRightWall(float val);
        void goToPose(float xg, float yg, float thetag);
    
        float getTheta() { return theta; }
        float getX() { return x; }
        float getY() { return y; }
        
};


#endif 