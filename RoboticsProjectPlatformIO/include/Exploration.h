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
      ESCAPE,
      ESCAPE_STOP,
      ESCAPE_REVERSE,
      ESCAPE_TURN,
      CELLY,
      CELLY_WAIT
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
        case CELLY: return "CELLY";
        case CELLY_WAIT: return "CELLY_WAIT";
        default: return "UNKNOWN";
      };
    }

    bool started = false;

    int pendingTurn = 0;
    int pickTurnDeg(float clearFL, float clearF, float clearFR);

    int frontBlockedCount = 0;
    int clearFCount = 0;
    int clearFLCount = 0;
    int clearFRCount = 0;

    const float NORTH = PI/2;
    const float SOUTH = -PI/2;

    float GOAL_DIR = NORTH;

    const float COLLISION = 16.0f;
    const float COLLISION_FL = 15.0f;
    const float COLLISION_FL_BAD = 6.0f;
    const float COLLISION_FR = 15.0f;
    const float COLLISION_FR_BAD = 6.0f;

    const float RIGHT_OPEN_THRESHOLD = 20.0f;
    const float WALL_DIST = 9.0f;

    // odom
    float lasX = 0.0f;
    float lastY = 0.0f;
    float lastTheta = 0.0f;
    int stuckCount =0;

    int rCount = 0;
    float chooseHeadingFromFrontArray(float FL, float F, float FR, float R, float IR_L, float IR_R);

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

    int committedTurnDirection = 0;
    uint8_t escapeStep = 0;

    float sidePenaltyFiltered = 0.0f;



    // MAP
    bool toCell(float x_mm, float y_mm, int &cx, int &cy);
    float visitedPenaltyForHeading(float x_mm, float y_mm, float headingRad);
    void markVisited(float x, float y);
    void markObstacles(float x, float y, float theta);
  
    static constexpr int CELL_MM = 100; // grid size

    // number of columns
    static constexpr int NX      = 14;
    static constexpr int NY      = 20;
    int GOAL    = CELL_MM * NY - 400; // first goal
    
    bool GOAL_1_CELLY = false;
    bool GOAL_2_CELLY = false;


    uint8_t map[NX][NY] = {0};

    int lastCellX = -1;
    int lastCellY = -1;

    float lastMarkX = 0.0f;
    float lastMarkY = 0.0f;

    void printVisitedGrid();

    bool wallCloseLeft = false;
    bool wallCloseRight = false;

    int avoidCount = 0;
    int lastAvoidTurn = 0;
    
    float turnStartTheta = 0.0f;
    int turnStuckCount = 0;

    float escapeStartX = 0.0f;
    float escapeStartY = 0.0f;
    int escapeStuckCount = 0;

    const int TURN_STUCK_LIMIT = 180;
    const int ESCAPE_STUCK_LIMIT = 180;

  public:
    Exploration(Sensors &sensors, Controllers &Controller);
    
    void startExploring();
    void stop();
    void update();
    
};

#endif