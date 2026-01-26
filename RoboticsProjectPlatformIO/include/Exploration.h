#ifndef EXPLORATION_H
#define EXPLORATION_H

#include <Arduino.h>
#include <Ultrasonics.h>
#include <Controller.h>
#include <Sensors.h>
#include <mbed.h>

class Exploration {
  public:
    Exploration(Sensors &sensors, Controller &controller);
    
    void startExploring();
    void stop();
    void update();

  private:
    // injected classes
    Sensors &sensors;
    Controller &controller;

    // parameters
    const float COLLISION = 16.0f;
    const float COLLISION_FL = 15.0f;
    const float COLLISION_FL_BAD = 6.0f;
    const float COLLISION_FR = 15.0f;
    const float COLLISION_FR_BAD = 6.0f;

    //stuck limits 
    static constexpr float STUCK_THRESHOLD = 0.5f;
    static constexpr float STUCK_LIMIT = 100;
    static constexpr int TURN_STUCK_LIMIT = 180;
    static constexpr int ESCAPE_STUCK_LIMIT = 180;

    // direction constants
    static constexpr float NORTH = PI/2;
    static constexpr float SOUTH = -PI/2;

    // state machine
    enum ExplorationStates {
      IDLE,           // 0
      TURN_TO_GOAL,   // 1
      TURNING,        // 2
      DRIVE,          // 3
      AVOID,          // 4
      WALL_FOLLOW,    // 5
      ESCAPE,
      CELLY,
      CELLY_WAIT
    };

    void handleTurnToGoal();
    void handleTurn(float theta, float F, float FL, float FR);
    void handleDrive(float y, bool hitC, bool hitL, bool hitR, bool critL, bool critR, float moved, float FL, float F, float FR, float R, float IR_L, float IR_R, float theta, float x);
    void handleAvoid(bool hitL, bool hitC, bool hitR, float x, float y, float theta, float FL, float F, float FR);
    void handleWallFollow();
    void handleEscape(float x, float y, float FL, float FR);
    void handleCelly();
    void handleCellyWait();

    ExplorationStates explorationState = IDLE;
    ExplorationStates prevState = IDLE;
    // ExplorationStates nextState = IDLE;
    void setState(ExplorationStates next);

    String stateToString(ExplorationStates state) {
      switch(state) {
        case IDLE: return "IDLE";
        case TURN_TO_GOAL: return "TURN_TO_GOAL";
        case TURNING: return "TURNING";
        case DRIVE: return "DRIVE";
        case AVOID: return "AVOID";
        case WALL_FOLLOW: return "WALL_FOLLOW";
        case CELLY: return "CELLY";
        case CELLY_WAIT: return "CELLY_WAIT";
        default: return "UNKNOWN";
      };
    }


    // ===== nav stuff =======
    float goalDirection = NORTH;
    int pendingTurn = 0;
    int committedTurnDirection = 0;

    // turn tracking
    float turnStartTheta = 0.0f;
    int turnStuckCount = 0;

    //escape sequence
    uint8_t escapeStep = 0;
    float escapeStartX = 0.0f;
    float escapeStartY = 0.0f;
    int escapeStuckCount = 0;

    // anti- left right at obstacle
    int avoidCount = 0;
    int lastAvoidTurn = 0;

    // movement tracking

    // odom
    float lasX = 0.0f;
    float lastY = 0.0f;
    float lastTheta = 0.0f;
    int stuckCount = 0;

    // ============= mapping ====================

    static constexpr int CELL_MM     = 100; // grid size
    static constexpr int MAP_COLUMNS = 14;
    static constexpr int MAP_ROWS    = 20;

    uint8_t map[MAP_COLUMNS][MAP_ROWS] = {0};

    float lastMarkX = 0.0f;
    float lastMarkY = 0.0f;

    // MAP
    bool toCell(float x_mm, float y_mm, int &cx, int &cy);
    float visitedPenaltyForHeading(float x_mm, float y_mm, float headingRad);
    void markVisited(float x, float y);
    void markObstacles(float x, float y, float theta);

    // ================ goal tracking =============== 
    int goalY    = CELL_MM * MAP_ROWS - 400; // first goal
    bool goal1Celly = false;
    bool goal2Celly = false;

    // nav logics
    float chooseHeadingFromFrontArray(float FL, float F, float FR, float R, float IR_L, float IR_R);

    int pickTurnDeg(float clearFL, float clearF, float clearFR);

    float wrapPi(float angle) {
      while (angle > PI) angle -= 2.0f * PI;
      while (angle < -PI) angle += 2.0f * PI;
      return angle;
    }

    void printVisitedGrid();

};

#endif