#ifndef EXPLORATION_H
#define EXPLORATION_H

#include <Arduino.h>
#include <Ultrasonics.h>
#include <Controllers.h>
#include <Sensors.h>
#include <mbed.h>

class Exploration {
  private:

    enum ExplorationStates {
      IDLE,           // 0
      TURN_TO_GOAL,   // 1
      TURNING,        // 2
      DRIVE,          // 3
      AVOID,          // 4
      WALL_FOLLOW,    // 5
      ESCAPE_STOP,
      ESCAPE_REVERSE,
      ESCAPE_TURN
    };

    String stateToString(ExplorationStates state) {
      switch(state) {
        case IDLE: return "IDLE";
        case TURN_TO_GOAL: return "TURN_TO_GOAL";
        case TURNING: return "TURNING";
        case DRIVE: return "DRIVE";
        case AVOID: return "AVOID";
        case WALL_FOLLOW: return "WALL_FOLLOW";
        case ESCAPE_STOP: return "ESCAPE_STOP";
        case ESCAPE_REVERSE: return "ESCAPE_REVERSE";
        case ESCAPE_TURN: return "ESCAPE_TURN";
        default: return "UNKNOWN";
      };
    }

    int pendingTurn = 0;
    int pickTurnDeg(float clearFL, float clearF, float clearFR);

    int frontBlockedCount = 0;
    int clearFCount = 0;
    int clearFLCount = 0;
    int clearFRCount = 0;

    float NORTH = PI/2;

    const float COLLISION = 16.0f;
    const float COLLISION_FL = 14.0f;
    const float COLLISION_FR = 14.0f;

    const float WALL_DIST = 9.0f;

    // odom
    float lasX = 0.0f;
    float lastY = 0.0f;
    float lastTheta = 0.0f;
    int stuckCount =0;

    float chooseHeadingFromFrontArray(float FL, float F, float FR);

    Sensors &sensors;
    Controllers &Controller;

    ExplorationStates explorationState = IDLE;
    ExplorationStates prevState = IDLE;
    ExplorationStates nextState = IDLE;
    void setState(ExplorationStates next);

    float wrapPi(float angle) {
            while (angle > PI) angle -= 2.0f * PI;
            while (angle < -PI) angle += 2.0f * PI;
            return angle;
        }

    bool targetInit = false;
    float targetFiltered = 0.0f;
  public:
    Exploration(Sensors &sensors, Controllers &Controller);
    
    void startExploring();
    void stop();
    void update();
    
};

#endif