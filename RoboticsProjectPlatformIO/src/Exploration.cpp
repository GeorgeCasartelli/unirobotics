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

    // rightWallThreshold = 15.0f;
    gapThreshold = 30.0f;
    // frontBlockedThreshold = 9.0f;
    alignAttempts = 0;

    leftTurnStreak = 0;

}

void Exploration::startExploring() {
    // setGoal(0,1680.0f, 0.0f);
    // setState(GOTO_GOAL);
    setState(WALL_FOLLOW);
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

        case WALL_FOLLOW:
            Serial.println("Starting wall follow");
            Controller.startWallFollowing(wallFollowingDist, 0.6f);
            break;

        case CORNER_TURN:
            //nothing to do
            break;

    }
}

void Exploration::setState(ExplorationStates next) {
    if (explorationState != next) {
        prevState = explorationState;
        explorationState = next;
        onEnterState();
    }
}

float Exploration::chooseEscapeTurn(float front, float fl, float fr) {
    const float OPEN_BIAS = 8.0f;
    float sideBias = fr - fl; // if +, right more open

    float theta = Controller.getTheta();

    if (sideBias > OPEN_BIAS) return Controller.getClosestCardinal(theta - PI/2);
    if (sideBias < -OPEN_BIAS) return Controller.getClosestCardinal(theta +  PI/2);

    // if 3 left turns in a row, return right turn
    if (leftTurnStreak >= 3) return Controller.getClosestCardinal(theta - PI/2);
    return Controller.getClosestCardinal(theta + PI/2);
}


void Exploration::frontBlocked(float front, float fl, float fr, float rightUS) {
    Controller.cancel();
    if (!Controller.isIdle()) { return; }
 
    float turn = chooseEscapeTurn(front, fl, fr);
    float theta = Controller.getTheta();

    if (rightUS > 15.0f) {
        turn = Controller.getClosestCardinal(theta - PI/2);
    }

    float delta = wrapPi(turn-theta);
    bool turningLeft = delta > 0;

    if (turningLeft) { leftTurnStreak++; }
    else { leftTurnStreak = 0; }

    Serial.println((String)"frontBlocked: theta=" + theta*(180/PI) +
                   " target=" + turn*(180/PI) +
                   " delta=" + delta*(180/PI));

    Controller.requestTurnToHeading(turn);
    
    setState(CORNER_TURN);
}


void Exploration::update() {

    sensors.update();
    // Serial.println((String)"front: " + sensors.getRightDist());

    float frontDist = sensors.getFrontDist();
    float rightDist = sensors.getRightDist();
    float avgRightDist = sensors.getRightAvg();
    float frontLeftDist = sensors.getFrontLeftDist();
    float frontRightDist = sensors.getFrontRightDist();


    // Controller.setWallDistance(rightDist_IR);
    
    RightDistances r = sensors.getRightDist_IR();
    Controller.setRightIR(r.front, r.rear);
    Controller.setRightUS(rightDist);
    
    // Serial.println((String)"Front Sensor Array: FL " + frontLeftDist + ", FR: " + frontRightDist + ", F: " + frontDist);
    Serial.println((String)"leftTurnStreak: " + leftTurnStreak);

    switch(explorationState) {
        case IDLE:
        break;

        case GOTO_GOAL: {

            if (Controller.isIdle()) {
                Serial.println("Controller stopped, setting to idle");
                setState(IDLE);
            }

            if (frontDist < frontBlockedThreshold) {
                Serial.println("RAAHHH OBSTACLE");
                Controller.cancel();
                setState(WALL_FOLLOW);
            }
            break;
        }


        case WALL_FOLLOW: {
            Serial.println((String)"WF: Front= " + frontDist + 
                              " RirhgtFront = "  + r.front + 
                              " FrontRight = "   + frontRightDist +
                              ", rightRead = "   + r.rear + 
                              ", rightUS = "     + rightDist);
            
            bool rightGap = (rightDist > gapThreshold) && canTriggerGap;
            gapCount = rightGap ? gapCount + 1 : 0;
            
            if (!canTriggerGap && rightDist < 10.0f) { 
                canTriggerGap = true;
            }

            if (frontDist < frontBlockedThreshold) {
  
                if (frontLeftDist > frontDist + 5.0f) {
                    Serial.println("Detected left corner");
                }
                frontBlocked(frontDist, frontLeftDist, frontRightDist, rightDist);
                break;
            }
            

            if (frontRightDist < frontRightBlockedThreshold) {
                Serial.println("FRONT RIGHT BLOCKED");
            }

            if (gapCount >= 5) {

                if (frontLeftDist < 35.0f && frontRightDist > 50.0f) {
                    Serial.println("Gap detected, FL/FR pattern suggests we are facing opening. Ignoring");
                    gapCount = 0;
                    break;

                }
                

                if (frontRightDist < 18.0f) {
                    Serial.println("Gap deteced but front right sees wall. Ignoring");
                    gapCount = 0;
                    break;
                } 

                Serial.println("REAL GAP ON RIGHT. TURNING");
                // Controller.moveDistance(160.0f, true);
                Controller.cancel();
                gapCount = 0;
                setState(PRE_GAP_ALIGN);
                canTriggerGap = false;
            }


            break;
        }

        case PRE_GAP_ALIGN: {
            if (Controller.isIdle()) {
                if (frontRightDist > 20.0f && avgRightDist < 15.0f) {
                    Controller.align();
                    setState(WAIT_ALIGN_MOVE);
                }
                if (frontRightDist > 25.0f) {

                    Controller.align();
                    setState(PREP_RIGHT_TURN);
                } else {
                    Controller.align();
                    setState(WAIT_ALIGN_MOVE);
                }
            }
            break;
        }

        case WAIT_ALIGN: {
            if (Controller.isIdle()) {
                setState(PREP_RIGHT_TURN);
            }
            break;
        }

        case WAIT_ALIGN_MOVE: {
            if (Controller.isIdle()) {
                Controller.moveDistance(160.0f, true);
                setState(PREP_RIGHT_TURN);
            }
            break;
        }

        case PREP_RIGHT_TURN: {
            if (frontDist < frontBlockedThreshold) {
                if (frontLeftDist > frontDist + 5.0f) {
                    Serial.println("Detected left corner");
                }
                frontBlocked(frontDist, frontLeftDist, frontRightDist, rightDist);;
                break;
            }
            if (Controller.isIdle()) {
                Controller.requestTurn(-90);
                setState(CORNER_TURN);
            }
            break;
        }

        case CORNER_TURN: {
            Serial.println("CORNER");
            if (Controller.isIdle()) {
                Serial.println("Turn done");
                if (prevState == PREP_RIGHT_TURN) {
                    Controller.moveDistance(80.0f, true);  
                    setState(POST_TURN_ESCAPE);               
                    
                } else if (prevState == POST_TURN_ESCAPE) {
                    Controller.moveDistance(60.0f, true);
                    setState(POST_CORNER_ALIGN);
                } else {
                    Controller.align();
                    setState(POST_CORNER_ALIGN);
                }
            }
            break;
        }

        case POST_CORNER_ALIGN: {
            if (Controller.isIdle()) {
                Controller.startWallFollowing(wallFollowingDist, 0.6f);
                setState(WALL_FOLLOW);
            }
            break;
        }


        case POST_TURN_ESCAPE: {
            Serial.println("POST_TURN_ESCAPE");

            // if (frontDist < frontBlockedThreshold) {
                if (rightDist > gapThreshold) {
                    if (Controller.isIdle()) {
                        Controller.requestTurn(-90);
                        setState(CORNER_TURN);
                        break;
                    }
                }
                if (frontLeftDist > frontDist + 5.0f) {
                    Serial.println("Detected left corner");
                }
                // frontBlocked();
            //     break;
            // }
            Controller.recalibrate();
            if (Controller.isIdle()){
                // leftTurnStreak = 
                leftTurnStreak = 0;
                setState(WALL_FOLLOW);
            }
            break;
        }
        case TEST:
        break;
    }


    Controller.update();
}