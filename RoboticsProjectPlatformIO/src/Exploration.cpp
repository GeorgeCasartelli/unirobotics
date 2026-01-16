#include <Exploration.h>
#include <Ultrasonics.h>
#include <Controllers.h>


Exploration::Exploration(Sensors &sensors, Controllers &controller)
    : sensors(sensors),        
      Controller(controller)
{
    explorationState = IDLE;
}

void Exploration::startExploring() {
    setState(TURN_TO_GOAL);
}


void Exploration::setState(ExplorationStates next){
    if (explorationState == next) return;
    prevState = explorationState;
    explorationState = next;
}

void Exploration::update(){
    sensors.update();

    float F = sensors.getFrontDist();
    float FL = sensors.getFrontLeftDist();
    float FR = sensors.getFrontRightDist();
    float R = sensors.getRightDist();

    bool hitL = FL < COLLISION;
    bool hitC = F < COLLISION;
    bool hitR = FR < COLLISION;

    bool wallFace = hitL && hitC && hitR;

    // bool frontBlocked = frontBlockedCount >= 3;

    
    // bool clearF = clearFCount >= 3;
    // bool clearFL = clearFLCount >= 3;
    // bool clearFR = clearFRCount >= 3;
    // bool rightWall = R < RIGHT_WALL;


    float x = Controller.getX();
    float y = Controller.getY();
    float theta = Controller.getTheta();

    float dx = x - lasX;
    float dy = y - lastY;
    float dtheta = theta - lastTheta;
    float moved = sqrtf(dx*dx + dy*dy);

    lasX = x;
    lastY = y;
    lastTheta = theta;

    const float STUCK_THRESH = 0.5f;
    const float STUCK_TURN_THRESH = 0.8f;
    const int STUCK_LIMIT = 100;

    
    Serial.println((String) "SEXPLORATION STATE: " + stateToString(explorationState));
    switch(explorationState) {
        case IDLE: {

            break;
        }
        case TURN_TO_GOAL: {
            if (Controller.isIdle()) {
                Serial.println("Turning to goal...");
                Controller.requestTurnToHeading(PI/2);
                setState(TURNING);
            }
            break;
        }

        case TURNING: {
            
            if (!Controller.isIdle()) break;

            Serial.println("Turning finished...");
            setState(DRIVE);

            break;
        }
        case DRIVE: {

            if (moved < STUCK_THRESH) {
                stuckCount++;
            } else stuckCount = 0;

            if (stuckCount > STUCK_LIMIT) {
                Controller.cancel();
                setState(ESCAPE_STOP);
                stuckCount = 0;
            }

            bool danger = hitC || hitL || hitR;
            if (danger) {
                Serial.println("Collision!!!");
                Controller.cancel();
                pendingTurn = pickTurnDeg(FL, F, FR);
                setState(AVOID);
                break;
            } 

            float target = chooseHeadingFromFrontArray(FL, F, FR);
            
            const float val = 0.88f;
            targetFiltered = wrapPi(val * targetFiltered + (1.0f - val) * target);

            Controller.driveHeading(targetFiltered, 0.60f);
            break;
        }

        case AVOID: {
            if (!Controller.isIdle()) break;
            if (pendingTurn == 90 && hitL && hitC && hitR) {
                // all 3 sensors blocked so facing a wall fully, start wall following.
                // Controller.startWallFollowing(WALL_DIST, 0.6f);
                // setState(WALL_FOLLOW);
                setState(ESCAPE_STOP);
                Serial.println("Set state to WALL FOLLOW");
            } else { 
                Controller.requestTurn(pendingTurn);
                setState(TURNING);
            }
            break;
        }

        case WALL_FOLLOW: {

            if (hitL || hitC || hitR) {
                Controller.cancel();
                pendingTurn = pickTurnDeg(FL, F, FR);
                wallFace = hitL && hitC && hitR;
                setState(AVOID);
            }
            break;
        }

        case ESCAPE_STOP: {
            if (!Controller.isIdle()) break;
            Controller.moveDistance(80.0, false);
            setState(ESCAPE_REVERSE);
            break;
        }


        case ESCAPE_REVERSE: {
            if (!Controller.isIdle()) break;

            static int lastEscapeTurn = 90;
            if (FR > FL + 5.0f) lastEscapeTurn = 90;
            else if (FL > FR + 5.0f) lastEscapeTurn = -90;
            Controller.requestTurn(lastEscapeTurn);
            setState(ESCAPE_TURN);
            break;
        }

        case ESCAPE_TURN: {
            if (!Controller.isIdle()) break;
            setState(DRIVE);
            break;
        }
    }

    Controller.update();
}

float Exploration::chooseHeadingFromFrontArray(float FL, float F, float FR) {
    float theta = Controller.getTheta();

    // headings of each sensor
    float headingL = wrapPi(theta + 30.0f * PI/180.0f);
    float headingF = theta;
    float headingR = wrapPi(theta - 30.0f * PI/180.0f);

    float clearL = FL < COLLISION_FL  ? -1000.0f   : FL;
    float clearF = F < COLLISION   ? -1000.0f   : F;
    float clearR = FR < COLLISION_FR  ? -1000.0f  : FR;


    // clear gaps were dominating so capping
    if (clearL > 120.0f) clearL = 120.0f;
    if (clearF > 120.0f) clearF = 120.0f;
    if (clearR > 120.0f) clearR = 120.0f;

    float gL = cosf(wrapPi(headingL - NORTH));
    float gF = cosf(wrapPi(headingF - NORTH));
    float gR = cosf(wrapPi(headingR - NORTH));

    const float weightCLear = 1.0f;
    const float weightGoal = 25.0f;

    // calculate "scores"
    float sFL = weightCLear * clearL + weightGoal * gL;
    float sF  = weightCLear * clearF + weightGoal * gF + 3.0f; // bias forward
    float sFR = weightCLear * clearR + weightGoal * gR;

    if (sFL >= sF && sFL >= sFR) return headingL;
    if (sFR >= sF && sFR >= sFL) return headingR;
    return theta;
}


int Exploration::pickTurnDeg(float FL, float F, float FR) {

    bool hitL = FL < COLLISION_FL;
    bool hitC = F < COLLISION;
    bool hitR = FR < COLLISION_FR;

    if (hitL && hitC && hitR) return 90;
    if (hitC && hitL)         return -90;
    if (hitC && hitR)         return 90;
    if (!hitC && hitL)        return -45;
    if (!hitC && hitR)        return 45;
    
    if (hitC) {
        return (FR > FL) ? -90 : 90;
    }
    return 0;
}