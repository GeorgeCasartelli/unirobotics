#ifndef EXPLORATION_H
#define EXPLORATION_J

#include <Arduino.h>
#include <Ultrasonics.h>
#include <Controllers.h>
#include <Sensors.h>
#include <mbed.h>

class Exploration {
  private:

    enum ExplorationStates {
      IDLE,
      GOTO_GOAL, 
      AVOID_OBSTACLE,
      WALL_FOLLOW, // right side
      PRE_GAP_ALIGN,
      WAIT_ALIGN,
      WAIT_ALIGN_MOVE,
      PREP_RIGHT_TURN,
      CORNER_TURN,
      POST_TURN_ESCAPE,
      POST_CORNER_ALIGN,
      ALIGN_TO_WALL,
      LEAVE_POINT_CHECK,
      ARRIVED,
      TEST
    };

    Sensors &sensors;
    Controllers &Controller;

    float poseX = 0.0f;
    float poseY = 0.0f;
    float poseTheta = 0.0f;

    float goalX = 0.0f;
    float goalY = 0.0f;
    float goalTheta = 0.0f;

    void setGoal(float x, float y, float theta);

    ExplorationStates explorationState;
    ExplorationStates prevState;

    bool controllerBusy;
    float gapDetectedDistance;
    float confirmationDistance;

    float gapThreshold;

    int alignAttempts;
    
    void onEnterState();
    void setState(ExplorationStates next);


    // bug stuff

    float mLineSLope;
    float mLineIntercept; // y intercept
    float hitPointX, hitPointY; // coords of obstacle hit
    bool followingWall;
    float distanceToGoalAtHit;

    //wall following

    float frontBlockedThreshold = 10.0f;
    float frontRightBlockedThreshold = 10.0f; 
    float wallFollowingDist = 10.0f;
    float Kp_wall = 0.05f;

    int gapCount = 0;
    bool canTriggerGap = true;

    int leftTurnStreak;
    float chooseEscapeTurn(float front, float fl, float fr);
    void frontBlocked(float front, float fl, float fr, float rightUS);


    float wrapPi(float angle) {
            while (angle > PI) angle -= 2.0f * PI;
            while (angle < -PI) angle += 2.0f * PI;
            return angle;
        }
  public:
    Exploration(Sensors &sensors, Controllers &Controller);
    
    void startExploring();
    void stop();
    void update();
    
};

#endif