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
    lastMarkX = Controller.getX();
    lastMarkY = Controller.getY();
    GOAL_DIR = NORTH;
    Controller.startPose();
    setState(TURN_TO_GOAL);
}



void Exploration::setState(ExplorationStates next){
    if (explorationState == next) return;
    prevState = explorationState;
    explorationState = next;
}

bool Exploration::toCell(float x_mm, float y_mm, int &cx, int &cy) {
    // update current x and y 
    cx = (int)floorf(x_mm / CELL_MM); // column 0-13
    cy = (int)floorf(y_mm / CELL_MM); // row 0-19

    if (cx < 0 || cx >= NX || cy < 0 || cy >= NY) return false; // out of bounds return false
    return true;
}


void Exploration::markVisited(float x, float y) {

    float dx = x - lastMarkX;
    float dy = y - lastMarkY;

    // mark after 50mm
    if (sqrtf(dx*dx + dy*dy) < 50.0f) return;

    int cx, cy;
    if (toCell(x, y, cx, cy)) {
        if (map[cx][cy] != 0) return; // if marked as
        map[cx][cy] = 1; // 1 is visited
    }

    lastMarkX = x;
    lastMarkY = y;
}

void Exploration::markObstacles(float x, float y, float theta) {

    float F = sensors.getFrontDist();
    
    if (F < 100.0f && F > 20.0f) {
        float obstX = x + F * cosf(theta) * 10;
        float obstY = y + F * sinf(theta) * 10;

        // Serial.println((String)"OBSTACLE @ " +obstX + ", "+ obstY);

        int cx, cy;
        if (toCell(obstX, obstY, cx, cy)) {
            map[cx][cy] = 2; // 2 is obstacle
        }
    }


}

float Exploration::visitedPenaltyForHeading(float x, float y, float headingRad) {
    const float LOOKAHEAD_MM = 250.0f;
    float x2 = x + LOOKAHEAD_MM * cosf(headingRad);
    float y2 = y + LOOKAHEAD_MM * sinf(headingRad);

    int cx, cy;
    if (!toCell(x2, y2, cx, cy)) {
        return 300.0f; // penalty if trying to go out of bounds
    }
    return (float)map[cx][cy];
}

void Exploration::printVisitedGrid() {
    for (int y = NY-1; y >= 0; --y) {
        for (int x = 0; x < NX; ++x) {
            if (map[x][y] == 2) Serial.print("#");
            else if (map[x][y] == 1) Serial.print("o");
            else Serial.print(".");
        }

        Serial.println(" ");
    }
    Serial.println("");
}


void Exploration::update(){
    
    sensors.update();


    float F = sensors.getFrontDist();
    float FL = sensors.getFrontLeftDist();
    float FR = sensors.getFrontRightDist();
    float R = sensors.getRightDist();

    float IR_L = sensors.getRightDist_IR().rear;
    float IR_R = sensors.getRightDist_IR().front;


    // Serial.println((String)"IR_L: " + IR_L + ", IR_R: " + IR_R);


    bool hitL = FL < COLLISION;
    bool hitC = F < COLLISION;
    bool hitR = FR < COLLISION;

    bool scaryHitL = FL < COLLISION_FL_BAD;
    bool scaryHitR = FR < COLLISION_FR_BAD;

    bool wallFace = hitL && hitC && hitR;


    float x = Controller.getX();
    float y = Controller.getY();
    float theta = Controller.getTheta();

    float dx = x - lasX;
    float dy = y - lastY;
    float dtheta = theta - lastTheta;
    float moved = sqrtf(dx*dx + dy*dy);

    /// mark cell move
    float dxm = x - lastMarkX;
    float dym = y - lastMarkY;

    markVisited(x, y);
    // markObstacles(x, y, theta);

    lasX = x;
    lastY = y;
    lastTheta = theta;

    const float STUCK_THRESH = 0.5f;
    const float STUCK_TURN_THRESH = 0.8f;
    const int STUCK_LIMIT = 100;

    static int dumpCounter = 0;
    dumpCounter++;
    if (dumpCounter % 20 == 0) {
        Serial.println("\r\n\r\n\r\n\r\n\r\n\r\n\r\n===============FRAME START=============\r\n\r\n");
        printVisitedGrid();

        Serial.println((String)"POSE: [       x: "+Controller.getX()+",        y: " + Controller.getY() + ",   theta: "+ Controller.getTheta()*(180/PI) + "    ]");
        Serial.println((String)"SEXPLORATION STATE: " + stateToString(explorationState));
        float align = cosf(wrapPi(theta - NORTH));
        Serial.println((String)"align=" + align + (align < 0 ? " (SOUTH-ish)" : " (NORTH-ish)"));

        Serial.println((String)"FL: "+FL+ ", F: "+ F + ", FR: "+FR+", R: " + R);
        Serial.println((String)"dxm: "+ dxm + ", dym:  "+ dym + ", lastMarkX: " + lastMarkX + ", lastMarkY: " + lastMarkY);
    }    

    // Serial.println(stateToString(explorationState));
    switch(explorationState) {
        case IDLE: {
            
            break;
        }
        case TURN_TO_GOAL: {
            if (Controller.isIdle()) {
                // Serial.println("Turning to goal...");
                // float base = Controller.getClosestCardinal(theta);
                // Controller.requestTurnToHeading(wrapPi(base + PI/2));
                Controller.requestTurnToHeading(GOAL_DIR);

                setState(TURNING);
            }
            break;
        }

        case TURNING: {

            if (prevState != TURNING) {
                turnStartTheta = theta;
                turnStuckCount = 0;
            }

            float turnProgress = fabsf(wrapPi(theta-turnStartTheta));

            if(turnProgress < 0.05) {
                turnStuckCount++;
            } else {
                turnStuckCount = 0;
            }
            
            if (turnStuckCount > TURN_STUCK_LIMIT) {
                Serial.println("STUCK WHILE TURNING. ESCAPING");
                Controller.cancel();
                turnStuckCount = 0;
                setState(ESCAPE);
                break;
            }
            
            if (!Controller.isIdle()) break;

            // Serial.println("Turning finished...");
            
            if (F > 30.0f && FL > 25.0f && FR > 25.0f) {
                committedTurnDirection = 0;
            }


            bool southish = cosf(wrapPi(theta - NORTH)) < -0.6f;

            if (avoidCount >= 2 && southish && F > 20.0f) {

                int confirmTurn = (lastAvoidTurn >= 0 ) ? 45 : -45;
                pendingTurn = confirmTurn;

                lastAvoidTurn = pendingTurn;
                // avoidCount++;

                setState(AVOID);
                break;
            }

            setState(DRIVE);

            break;
        }
        case DRIVE: {

            if (y > GOAL && GOAL_DIR == NORTH) {
                Serial.println("HIT GOAL 1");
                Controller.cancel();
                setState(CELLY);
                break;
            }
            else if (y < GOAL && GOAL_DIR == SOUTH) {
                Serial.println("HIT GOAL 2");
                Controller.cancel();
                setState(CELLY);
                break;
            }

            bool danger = hitC || (hitL&&hitC) || (hitR&&hitC) || scaryHitL || scaryHitR;
            if (danger) {
                // Serial.println("Collision!!!");
                Controller.cancel();

                if (committedTurnDirection == 0) {
                    committedTurnDirection = pickTurnDeg(FL, F, FR);
                    // Serial.println((String)"Commiting to turning: " + committedTurnDirection);
                }
                pendingTurn = committedTurnDirection;
                
                avoidCount++;
                lastAvoidTurn = pendingTurn;
                setState(AVOID);
                break;
            } 

            if (moved < STUCK_THRESH) {
                stuckCount++;
            } else stuckCount = 0;

            if (stuckCount > STUCK_LIMIT) {
                Controller.cancel();
                setState(ESCAPE);
                stuckCount = 0;
                break;
            }
            
            if (F > 30.0f && FR > 25.0f && FL > 25.0f) {
                avoidCount = 0;
            }

            float target = chooseHeadingFromFrontArray(FL, F, FR, R, IR_L, IR_R);

            
            // const float val = 0.85f;
            // targetFiltered = wrapPi(val * targetFiltered + (1.0f - val) * target);

            // Controller.driveHeading(targetFiltered, 0.60f);
            Controller.driveHeading(target, 0.6f);
            
            break;
        }

        case AVOID: {
            if (!Controller.isIdle()) break;


            // mark obstacle
            if (FL < COLLISION_FL) {
                float obstX = x + 120 * cosf(theta + 30*PI/180);
                float obstY = y + 120 * sinf(theta - 30*PI/180);
                int cx,cy;
                if (toCell(obstX, obstY, cx, cy)) {
                    map[cx][cy] = 2;
                }
            }
            if (F < COLLISION) {
                float obstX = x + 120 * cosf(theta);
                float obstY = y + 120 * sinf(theta);
                int cx,cy;
                if (toCell(obstX, obstY, cx, cy)) {
                    map[cx][cy] = 2;
                }
            }
            if (FR < COLLISION_FR) {
                float obstX = x + 120 * cosf(theta);
                float obstY = y + 120 * sinf(theta);
                int cx,cy;
                if (toCell(obstX, obstY, cx, cy)) {
                    map[cx][cy] = 2;
                }
            }
            if (pendingTurn == 90 && hitL && hitC && hitR) {
                setState(ESCAPE);
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

        case ESCAPE: {
            if (prevState != ESCAPE) {
                escapeStartX = x;
                escapeStartY = y;
                escapeStuckCount = 0;
            }

            float escapeDist = sqrt(
                (x - escapeStartX) * (x - escapeStartX) +
                (y - escapeStartY) * (y - escapeStartY)
            );

            if (escapeDist < 0.5f) {
                escapeStuckCount++;
            } else {
                escapeStuckCount = 0;
                escapeStartX = x;
                escapeStartY = y;
            }

            if (escapeStuckCount > ESCAPE_STUCK_LIMIT) {
                Serial.println("STUCK WHILE ESCAPING SHIT");
                Controller.cancel();
                escapeStep = 0;
                escapeStuckCount = 0 ;
                avoidCount = 0;
                committedTurnDirection = 0;


                int randomTurn = (((int)x + (int)y) % 2) ? 135 : -135;
                Controller.requestTurn(randomTurn);
                setState(TURNING);
                break;
            }

            if (escapeStep == 0 && Controller.isIdle()) {
                Controller.moveDistance(80.0, false);
                escapeStep = 1;
            }
            else if (escapeStep == 1 && Controller.isIdle()) {

                if (FL > FR) {
                    Controller.requestTurn(-45);
                } else Controller.requestTurn(45);

                escapeStep = 2;
            }
            else if (escapeStep == 2 && Controller.isIdle()) {
                setState(DRIVE);
                escapeStep = 0;
            }
            break;
}

        case CELLY: {
            if (!Controller.isIdle()) break;

            if (GOAL_DIR== NORTH && !GOAL_1_CELLY) {

                Controller.requestTurn(1080);
                GOAL_1_CELLY = true;
                
                setState(CELLY_WAIT);
                break;
            }

            if (GOAL_DIR == SOUTH && !GOAL_2_CELLY) {
                // Controller.requestTurn(720);
                Controller.cancel();
                GOAL_2_CELLY = true;
                setState(IDLE);
                break;
            }
            break;
        }

        case CELLY_WAIT: {
            Serial.println("CELLY_WAIT");

            if (!Controller.isIdle()) break;

            // if (cellySpins > 0) {
            //     Controller.requestTurn(360);
            //     cellySpins--;
            //     break;
            // }

            GOAL_DIR = SOUTH;
            GOAL = 150;

            setState(TURN_TO_GOAL);
            break;
        }
    }

    Controller.update();
}



float Exploration::chooseHeadingFromFrontArray(float FL, float F, float FR, float R, float IR_L, float IR_R) {

    const float FL_CLOSE = 40.0f;
    const float F_CLOSE  = 40.0f;
    const float FR_CLOSE = 40.0f;
    const float OBSTACLE_PENALTY = -120.0f;
    float theta = Controller.getTheta();
    
    // Three options: turn left 40°, straight, turn right 40°
    float turnAngle = 40.0f * PI/180.0f;
    float headingL = wrapPi(theta + turnAngle);
    float headingF = theta;
    float headingR = wrapPi(theta - turnAngle);

    static float lastHeading = 0.0f;
    const float HYSTERESIS_BONUS = 30.0f;


    const float SIDE_PANIC = 20.0f;
    if (FL < SIDE_PANIC) return headingR;
    if (FR < SIDE_PANIC) return headingL;

    float goalWeight = 80.0f;
    float goalL = cosf(headingL - GOAL_DIR);
    float goalF = cosf(headingF - GOAL_DIR);
    float goalR = cosf(headingR - GOAL_DIR);

    // float southPunishL = goalL < -0.5 ? OBSTACLE_PENALTY : 0.0f;
    // float southPunishF = goalF < -0.5 ? OBSTACLE_PENALTY : 0.0f;
    // float southPunishR = goalF < -0.5 ? OBSTACLE_PENALTY : 0.0f;

    // if too close then punish. 
    // if gap AND facing goal, reward dist*2
    // else just return dist
    // clearL = FL < FL_CLOSE ? OBSTACLE_PENALTY : (  (goalL > 0) ? (FL * 1.4) : FL  );
    // clearF = F  < F_CLOSE  ? OBSTACLE_PENALTY : (  (goalF > 0) ? (F  * 1.4) : F   );
    // clearR = FR < FR_CLOSE ? OBSTACLE_PENALTY : (  (goalR > 0) ? (FR * 1.4) : FR  );

    float clearL = FL < FL_CLOSE ? OBSTACLE_PENALTY : FL;
    float clearF = F  < F_CLOSE  ? OBSTACLE_PENALTY : F;
    float clearR = FR < FR_CLOSE ? OBSTACLE_PENALTY : FR;

    clearL = min(clearL, 160.0f);
    clearF = min(clearF, 160.0f);
    clearR = min(clearR, 160.0f);

    // steer away if IR too close
    float steerL = IR_R < 15.0f ? 200.0f : 0.0f;
    float steerR = IR_L < 15.0f ? 200.0f : 0.0f;

    
    const float SIDE_NEAR = 110.0f;
    float minSide = min(FL, FR);
    if (minSide < SIDE_NEAR) {
        // stronger penalty the closer you are to a side wall
        clearF -= (SIDE_NEAR - minSide);   // cheap, linear, effective
    }

    // Serial.println((String)"clearL: "+clearL+", clearF"+clearF+", clearR: "+ clearR);

    float sFL = clearL + goalL * goalWeight + steerL;
    float sF  = clearF + goalF * goalWeight;
    float sFR = clearR + goalR * goalWeight + steerR;

    if (FL > 100.0f && FR > 100.0f && FL > 100.0f) {
        sF += 80.0f;
    }

    // if (goalL < -0.6) sFL -= 50.0f;
    // if (goalF < -0.6) sF  -= 50.0f;
    // if (goalR < -0.6) sFR -= 50.0f;

    float deltaL = fabsf(wrapPi(headingL - lastHeading));
    float deltaF = fabsf(wrapPi(headingF - lastHeading));
    float deltaR = fabsf(wrapPi(headingR - lastHeading));
    
    if (deltaL < 0.1f) sFL += HYSTERESIS_BONUS;  // Was going left
    if (deltaF < 0.1f) sF  += HYSTERESIS_BONUS;  // Was going straight
    if (deltaR < 0.1f) sFR += HYSTERESIS_BONUS;  // Was going right

    float headingChoice;
    if (sFL >= sF && sFL >= sFR) {
        // Serial.println((String)"headingL. score: "+sFL);
        headingChoice = headingL;
    }
    else if (sFR >= sF && sFR >= sFL) {
        // Serial.println((String)"headingR, score: "+sFR );
        headingChoice = headingR;
    } else {
        // Serial.println((String)"headingF. score: "+ sF);
        headingChoice = headingF;
    }

    lastHeading = headingChoice;
    return headingChoice;

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
        const float MIN_DIFF = 8.0f;
        if (FR > FL + MIN_DIFF) return -90;  // right clearly better
        if (FL > FR + MIN_DIFF) return 90;   // left clearly better
        
        // if similar, pick bigger one
        return (FR > FL) ? -90 : 90;
    }
    return 0;
}