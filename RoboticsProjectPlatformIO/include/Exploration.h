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

    float rightWallThreshold;
    float gapThreshold;
    float frontBlockedThreshold;

    int alignAttempts;
    
    void onEnterState();
    void setState(ExplorationStates next);

  public:
    Exploration(Sensors &sensors, Controllers &Controller);
    
    void startExploring();
    void stop();
    void update();
    
};

#endif