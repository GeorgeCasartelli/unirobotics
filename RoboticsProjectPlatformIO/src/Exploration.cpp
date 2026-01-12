#include <Exploration.h>
#include <Ultrasonics.h>
#include <Controllers.h>


Exploration::Exploration(Sensors &sensors, Controllers &controller)
    : sensors(sensors),        
      Controller(controller)
{
    explorationState = IDLE;
    prevState = IDLE;
    controllerBusy = false;

    gapDetectedDistance = 0.0f;
    confirmationDistance = 100.0f;

    rightWallThreshold = 15.0f;
    gapThreshold = 25.0f;
    frontBlockedThreshold = 12.0f;
    alignAttempts = 0;

}

void Exploration::startExploring() {
    setGoal(0,300.0f, 0.0f);
    setState(GOTO_GOAL);
    // Controller.moveContinuous(true); 
}

void Exploration::stop() {
    setState(IDLE);
}

void Exploration::setGoal(float x, float y, float theta) {
    goalX = x;
    goalY = y;
    goalTheta = theta;
}

void Exploration::onEnterState() {
    switch(explorationState) {
        case IDLE:
            break;

        case GOTO_GOAL:
            Serial.println((String)"Entering gotogoal, triggering controller: x: "+goalX+", y: " + goalY);
            Controller.goToPose(goalX, goalY, goalTheta);
            break;

        case AVOID_OBSTACLE:
            Controller.turnDegrees(90);
    }
}

void Exploration::setState(ExplorationStates next) {
    if (explorationState != next) {
        prevState = explorationState;
        explorationState = next;
        onEnterState();
    }
}
void Exploration::update() {

    sensors.update();
    Serial.println((String)"front: " + sensors.getRightDist());

    switch(explorationState) {
        case IDLE:
        break;

        case GOTO_GOAL: {

            if (Controller.isIdle()) {
                Serial.println("Controller stopped, setting to idle");
                setState(IDLE);
            }

            if (sensors.getFrontDist() < 15) {
                Serial.println("RAAHHH OBSTACLE");
                setState(AVOID_OBSTACLE);
            }
            break;
        }


        case AVOID_OBSTACLE: {
            
            break;
        }

        case TEST:
        break;
    }


    Controller.update();
}