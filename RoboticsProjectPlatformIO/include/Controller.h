#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <mbed.h>
#include <Arduino.h>
#include <Motors.h>
#include <Gyro.h> // not used 


class Controller{
    public:

        Controller(Motors &motor, Gyro &gyro); 

        void update();

        //motion ctrl
        void moveDistance(float target, bool forward);
        void requestTurn(float angle);
        void requestTurnToHeading(float target);      
        void driveHeading(float targetHeadingRad, float baseSpeed);
        void moveContinuous(bool forward, float speed = 0.7f);
        void cancel();


        // wall following logic
        void followWall();
        void startWallFollowing(float targetDist, float speed);
        void setWallDistance(float distance);
        void requestWallAlign();   

        // nav
        void goToPose(float xg, float yg, float thetag);
        void startPose();
        void align();

        // state queries
        bool isIdle();
        float getAvgDistance();

        float getClosestCardinal(float theta);

        void recalibrate();

        //wall follow
        void setRightIR(float front, float rear) { rightFrontIR = front; rightRearIR = rear;}
        void setRightUS(float us) { rightUS = us; }


        float getTheta() const { return theta; }
        float getX() const { return x; }
        float getY() const { return y; }
        
    
    
    private:
        // ====== hardware =========
        Motors &motors;
        Gyro &gyro;
        
        
        // control params
        
        static constexpr float minPWM = 0.3f;
        static constexpr float KP = 0.01f;

        // turning
        static constexpr float KP_TURN = 0.8f;
        static constexpr float TURN_TOLERANCE = 0.01;
        static constexpr float MAX_TURN_SPEED = 0.5f;


        // distance movmnt
        static constexpr float KP_DIST = 0.005f;
        static constexpr float DIST_TOLERANCE = 0.0005;
        
        // wall follow
        static constexpr float KP_WALL_ANGLE =  0.06f;
        static constexpr float KP_WALL_DIST = 0.01f;
        static constexpr float WALL_PRESENT = 20.0f;
        static constexpr float WALL_CORRECTION = -0.12f;
        static constexpr float WALL_SPEED_MIN = 0.25f;
        static constexpr float WALL_FILTER = 0.8f;
 
        // heading consts
        static constexpr float K_HEADING = 0.22f;
        static constexpr float MAX_STEER = 0.22f;

        // aligns
        static constexpr uint8_t ALIGN_TIMEOUT = 100;
        static constexpr float ALIGN_TOLERANCE = 0.05f;
        static constexpr float ALIGN_SPEED = 0.2f;

        // ====== state machine ======
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

        enum GoToSTATES {
            NONE,
            TURN1,
            DRIVE,
            DONE
        };

        STATES controllerState = IDLE;
        STATES prevState = IDLE;
        GoToSTATES gotoStep = NONE;

        void setState(STATES next);
        void onEnterState();
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

        // state handlers

        void handleMoving(float ds);
        void handleTurnPrep();
        void handleTurning(float dtheta);
        void handleWallFollowing();
        void handleAlignToWall();
        void handleHeadingDrive();

        bool printStates = false;

        // movement

        float distanceTarget = 0.0f;
        float startDistanceA = 0.0f;
        float startDistanceB = 0.0f;
        bool movingForward = true;

        float turnTargetDeg = 0.0f;
        float turnTarget = 0.0f;
        float pendingTurnTarget = 0.0f;
        float prevTurnError = 0.0f;
        bool turnErrorInit = false;

        int leftSign = 1;
        int rightSign = 1;

        float targetHeading = 0.0f;
        float headingDriveSpeed = 0.6f;
        
        // robot pose
        float x = 0.0f;
        float y = 0.0f;
        float theta = 0.0f;

        float prevDistA = 0.0f;
        float prevDistB = 0.0f;

        void resetPose();
        void updatePose();
        
        bool obstacleDetected;

        const float trackWidth = 110.0f;

        
        //goto
        float goalX = 0.0f;
        float goalY = 0.0f;
        float goalTheta = 0.0f;
        float goToHeading = 0.0f;
        float goToDistance = 0.0f;
        
        void runGoToStep();

        // wall following

        float currentWallDistance = 0.0f;
        float baseWallSpeed = 0.5f;
        float filteredWallDistance = 0.0f;
        float targetWallDistance = 12.0f;

        float calculateTrapezoidalSpeed(float traveled, float totalDistance, float maxSpeed);

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

};


#endif 