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
      LOCATING_WALL,
      APPROACHING_WALL,
      ALIGN_WITH_WALL,
      FOLLOWING_WALL,
      GAP_DETECTED,
      TURNING_RIGHT,
      TURNING_LEFT,
      FINISHED
    };

    Sensors &sensors;
    Controllers &Controller;

    ExplorationStates explorationState;
    ExplorationStates prevState;

    bool controllerBusy;
    float gapDetectedDistance;
    float confirmationDistance;

    float rightWallThreshold;
    float gapThreshold;
    float frontBlockedThreshold;
    

    void setState(ExplorationStates state);

  public:
    Exploration(Sensors &sensors, Controllers &Controller);
    
    void startExploring();
    void update();
    
};

#endif