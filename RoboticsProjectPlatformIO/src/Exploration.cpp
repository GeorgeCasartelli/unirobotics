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
    explorationState = TEST;
    // Controller.moveContinuous(true); 
}

void Exploration::setState(ExplorationStates state) {
    prevState = explorationState;
    explorationState = state;
}
void Exploration::update() {
    sensors.update();
    float frontDist = sensors.getFrontDist();
    float rightDist = sensors.getRightDist();
    
    RightDistances right = sensors.getRightDist_IR();
    

    float rightFront = right.front;
    float rightRear = right.rear;

    bool frontBlocked = frontDist < frontBlockedThreshold;
    bool rightGapExists = rightDist > gapThreshold;
    bool rightWallNear = rightDist < rightWallThreshold;

    float rightAvg = sensors.getRightAvg();
    // Serial.println((String)"State: "+ explorationState);
    // Serial.println((String)"Frontblocked: "+ frontBlocked + " frostDist: " + frontDist + " rightGapExists: " + rightGapExists + " rightWallNear: " + rightWallNear);

    Controller.update();

    switch(explorationState) {
        case IDLE: {
            break;
        }

        case LOCATING_WALL: {
            // Serial.println((String)"frontDist: " + frontDist + " rightDist: " + rightDist + " leftDist: " + leftDist);
            // 

            if (rightDist < frontDist) {
                if (rightDist < rightWallThreshold) {
                    Serial.println("Right wall close enough");
                    setState(FOLLOWING_WALL);
                }
                else {
                    Serial.println("We need to turn right");
                    setState(TURNING_RIGHT);
                }
            } else if (rightDist >= frontDist) {
                Serial.println("Front is closer");
                Controller.moveContinuous(true);
                setState(APPROACHING_WALL);
            } 
            break;
        }
        
        case APPROACHING_WALL: {
            if (frontDist < frontBlockedThreshold) {
                Serial.println("Front blocked!! Turning left");
                setState(TURNING_LEFT);
            }
            break;
        }

        case ALIGN_WITH_WALL: {
            Serial.println((String)"Lets align!! rightFront: " + rightFront + " rightRear: " + rightRear);

            float alignmentError = rightFront - rightRear;
            float alignmentTolerance = 1.0f; // 1cm tolerance

            if (fabs(alignmentError) < alignmentTolerance || alignAttempts >= 10) {
                Serial.println((String)"Aligned! alignAttempts: " + alignAttempts);
                alignAttempts = 0;
                setState(FOLLOWING_WALL);
                Controller.moveContinuous(true);
                break;
            }
            if (!controllerBusy) {
                float turnAngle = alignmentError * 0.05f;

                if (fabs(turnAngle) > 2.0f) {
                    Controller.turnDegrees(-turnAngle); // negative as value is opposite to alignment sign
                    controllerBusy = true;
                    alignAttempts++;
                }

            }
            else if (Controller.isIdle()) {
                controllerBusy = false;
            }
            break;
        }

        case FOLLOWING_WALL: {
            if (frontBlocked && rightGapExists) {
                Serial.println("TURNING RIGHT");
                explorationState = TURNING_RIGHT;
            }
            else if (frontBlocked) {
                Serial.println("TURNING LEFT");
                explorationState = TURNING_LEFT;
            }
            // else if (rightGapExists) {
            //     Serial.println("Gap on right!");
            //     gapDetectedDistance = Controller.getAvgDistance();
            //     explorationState = GAP_DETECTED;
            // }
            
            Serial.println((String)"RightAvg is: " + rightAvg);
            // Serial.println((String)rightFront);
            // Controller.moveContinuous(true); // should just run on its own 
            Controller.updateRightWall(rightAvg);
        } break;

        case TEST: {
            // Serial.println((String)"Front: " + rightFront + " Rear: " + rightRear);
            
            break;
        }
        case GAP_DETECTED: {

        } break;
            

        case TURNING_RIGHT:{

            break;
        }
        case TURNING_LEFT:{

            break;
        }

        case FINISHED:{
            Serial.println("FINISHED");
            break;
        }
        

    }


}

