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
        float pendingTurnTarget;
        float startDistanceA;
        float startDistanceB;
        float movingForward;
        
        float startAngle;
        
        const float maxTurnSpeed = 0.5f;

        bool obstacleDetected;

        const float trackWidth = 110.0f;

        const float Kp = 0.01;
        const float Kp_turn = 0.8f;
        const float Kp_dist = 0.005f;
        const float Kp_angle =  0.06f;

        const float tolerance = 0.0005;
        const float turnTolerance = 0.01;
        const float minPWM = 0.3f;

        float prevTurnError = 0.0f;
        bool turnErrorInit = false;

        int leftSign = 1;
        int rightSign = 1;

        // robot local pose
        float x = 50.0;
        float y = 50.0;
        float theta = 0.0;

        float prevDistA;
        float prevDistB;

        void resetPose();
        void updatePose();

        void runGoToStep();
        // STATES
        bool printStates = false;

        enum STATES {
            IDLE,
            MOVING,
            TURN_PREP,
            TURNING,
            GOTO,
            WALL_FOLLOWING,
            ALIGN_TO_WALL,
            HEADING_DRIVE
        };

        float targetWallDistance;
        float currentWallDistance;
        float Kp_wall = 0.01f;
        float baseWallSpeed = 0.5f;


        float K_heading = 0.22f;
        float maxSteer = 0.22f;


        float wallDistanceHistory[3];
        int wallHistoryIndex;
        float filteredWallDistance;

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

        void setState(STATES next);
        void onEnterState();

        float calculateTrapezoidalSpeed(float traveled, float totalDistance, float maxSpeed);
        
        Motors &motors;
        Gyro &gyro;

        const float TARGET_WALL_DIST = 1500.0f;

        float rightDistanceAvg; 

        const char* stateToString(STATES s) {
            switch (s) {
                case IDLE: return "IDLE";
                case MOVING: return "MOVING";
                case TURNING: return "TURNING";
                case TURN_PREP: return "TURN_PREP";
                case GOTO: return "GOTO";
                case WALL_FOLLOWING: return "WALL_FOLLOWING";
                case ALIGN_TO_WALL: return "ALIGN_TO_WALL";
                case HEADING_DRIVE: return "HEADING_DRIVE";
                default: return "UNKNOWN";
            };
        }
        
        float wrapPi(float angle) {
            while (angle > PI) angle -= 2.0f * PI;
            while (angle < -PI) angle += 2.0f * PI;
            return angle;
        }

        float rightFrontIR = 0;
        float rightRearIR = 0;
        float rightUS = 0;
        bool distLocked = false;

        const float LOCK_IN = 0.8f;
        const float LOCK_OUT = 1.8f;

        int alignCount = 0;

        
    public:

        void startPose();
        void align();
        void driveHeading(float targetHeadingRad, float baseSpeed);

        Controllers(Motors &motor, Gyro &gyro); 
        void moveDistance(float target, bool forward);
        void requestTurn(float angle);
        void update();
        void setObstacleDetected(bool flag);
        bool isIdle();
        float getAvgDistance();
        void goToPose(float xg, float yg, float thetag);
        float getClosestCardinal(float theta);
        void requestTurnToHeading(float target);
        void requestTurnLeftToCardinal();
        void requestTurnRightToCardinal();
        void requestWallAlign();
        void recalibrate();

        //wall follow
        void followWall();
        void moveContinuous(bool forward, float speed = 0.7f);
        void startWallFollowing(float targetDist, float speed);
        void setWallDistance(float distance);

        void setRightIR(float front, float rear) { rightFrontIR = front; rightRearIR = rear;}
        void setRightUS(float us) { rightUS = us; }


        float getTheta() { return theta; }
        float getX() { return x; }
        float getY() { return y; }
        
        void cancel();
};


#endif 