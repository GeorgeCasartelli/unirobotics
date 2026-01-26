#include <Exploration.h>
#include <Ultrasonics.h>
#include <Controller.h>


Exploration::Exploration(Sensors &sensors, Controller &controller)
    : sensors(sensors),        
      controller(controller)
{
    explorationState = IDLE;
}

// public api

void Exploration::startExploring() {
    lastMarkX = controller.getX();
    lastMarkY = controller.getY();
    goalDirection = NORTH;
    controller.startPose();
    setState(TURN_TO_GOAL);
}

/// MAP PRINTING

void Exploration::printVisitedGrid() {
    for (int y = MAP_ROWS-1; y >= 0; --y) {
        for (int x = 0; x < MAP_COLUMNS; ++x) {
            if (map[x][y] == 2) Serial.print("#");
            else if (map[x][y] == 1) Serial.print("o");
            else Serial.print(".");
        }

        Serial.println(" ");
    }
    Serial.println("");
}



//  =============== UPDATE LOOP  +++++=============



void Exploration::update(){
    
    sensors.update();

    // read sensors
    float F = sensors.getFrontDist();
    float FL = sensors.getFrontLeftDist();
    float FR = sensors.getFrontRightDist();
    float R = sensors.getRightDist();

    float IR_L = sensors.getRightDist_IR().left;
    float IR_R = sensors.getRightDist_IR().right;

    // collision detection
    bool hitL = FL < COLLISION;
    bool hitC = F < COLLISION;
    bool hitR = FR < COLLISION;
    bool scaryHitL = FL < COLLISION_FL_BAD;
    bool scaryHitR = FR < COLLISION_FR_BAD;

    bool wallFace = hitL && hitC && hitR;

    // get current pose
    float x = controller.getX();
    float y = controller.getY();
    float theta = controller.getTheta();

    // movement since last update 
    float dx = x - lasX;
    float dy = y - lastY;
    float dtheta = theta - lastTheta;
    float moved = sqrtf(dx*dx + dy*dy);

    /// mark cell move

    markVisited(x, y);
    // markObstacles(x, y, theta);

    lasX = x;
    lastY = y;
    lastTheta = theta;

    // debug/map 
    static int dumpCounter = 0;
    dumpCounter++;
    if (dumpCounter % 20 == 0) {
        Serial.println("\r\n\r\n\r\n\r\n\r\n\r\n\r\n===============FRAME START=============\r\n\r\n");
        printVisitedGrid();

        Serial.println((String)"POSE: [       x: "+controller.getX()+",        y: " + controller.getY() + ",   theta: "+ controller.getTheta()*(180/PI) + "    ]");
        Serial.println((String)"SEXPLORATION STATE: " + stateToString(explorationState));
        float align = cosf(wrapPi(theta - NORTH));
        Serial.println((String)"align=" + align + (align < 0 ? " (SOUTH-ish)" : " (NORTH-ish)"));

        Serial.println((String)"FL: "+FL+ ", F: "+ F + ", FR: "+FR+", R: " + R);
        Serial.println((String)"lastMarkX: " + lastMarkX + ", lastMarkY: " + lastMarkY);
    }    

    // Serial.println(stateToString(explorationState));
    switch(explorationState) {
        case IDLE: {
            
            break;
        }
        case TURN_TO_GOAL: {
            handleTurnToGoal();
            break;
        }

        case TURNING: {
            handleTurn(theta, F, FL, FR);
            break;
        }
        case DRIVE: {

            handleDrive(y, hitC, hitL, hitR, scaryHitL, scaryHitR, moved, FL, F, FR, R, IR_L, IR_R, theta, x);
            break;
        }

        case AVOID: {
            handleAvoid(hitL, hitC, hitR, x, y, theta, FL, F, FR);
            break;
        }

        case ESCAPE: {
            handleEscape(x, y, FL, FR);
            break;
        }

        case CELLY: {
            handleCelly();
            break;
        }

        case CELLY_WAIT: {
            handleCellyWait();
            break;
        }
    }

    controller.update();
}

// ============================= SECTION ============================
// ============================  UPDATE HANDLERS ===========================

void Exploration::handleTurnToGoal() {
    if (controller.isIdle()) return;

    controller.requestTurnToHeading(goalDirection);
    setState(TURNING);    
}

void Exploration::handleTurn(float theta, float F, float FL, float FR) {
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
        controller.cancel();
        turnStuckCount = 0;
        setState(ESCAPE);
        return;
    }
    
    if (!controller.isIdle()) return;

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
        return;
    }

    setState(DRIVE);
}

void Exploration::handleDrive(float y, bool hitC, bool hitL, bool hitR, bool critL, bool critR, float moved, float FL, float F, float FR, float R, float IR_L, float IR_R, float theta, float x){

    if (y > goalY && goalDirection == NORTH) {
        Serial.println("HIT GOAL 1");
        controller.cancel();
        setState(CELLY);
        return;
    }
    else if (y < goalY && goalDirection == SOUTH) {
        Serial.println("HIT GOAL 2");
        controller.cancel();
        setState(CELLY);
        return;
    }

    bool danger = hitC || (hitL&&hitC) || (hitR&&hitC) || critL || critR;
    if (danger) {
        // Serial.println("Collision!!!");
        controller.cancel();

        if (committedTurnDirection == 0) {
            committedTurnDirection = pickTurnDeg(FL, F, FR);
            // Serial.println((String)"Commiting to turning: " + committedTurnDirection);
        }
        pendingTurn = committedTurnDirection;
        
        avoidCount++;
        lastAvoidTurn = pendingTurn;
        setState(AVOID);
        return;
    } 

    if (moved < STUCK_THRESHOLD) {
        stuckCount++;
    } else stuckCount = 0;

    if (stuckCount > STUCK_LIMIT) {
        controller.cancel();
        setState(ESCAPE);
        stuckCount = 0;
        return;
    }
    
    if (F > 30.0f && FR > 25.0f && FL > 25.0f) {
        avoidCount = 0;
    }

    float target = chooseHeadingFromFrontArray(FL, F, FR, R, IR_L, IR_R);

    controller.driveHeading(target, 0.6f);
}
    
void Exploration::handleAvoid(bool hitL, bool hitC, bool hitR, float x, float y, float theta, float FL, float F, float FR){
    if (!controller.isIdle()) return;

    // mark obstacle on map
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

    // (reverse a bit using escape logic)
    if (pendingTurn == 90 && hitL && hitC && hitR) {
        setState(ESCAPE);
    } else { 
        controller.requestTurn(pendingTurn);
        setState(TURNING);
    }
}

void Exploration::handleWallFollow(){

}
void Exploration::handleEscape(float x, float y, float FL, float FR){
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
        controller.cancel();
        escapeStep = 0;
        escapeStuckCount = 0 ;
        avoidCount = 0;
        committedTurnDirection = 0;


        int randomTurn = (((int)x + (int)y) % 2) ? 135 : -135;
        controller.requestTurn(randomTurn);
        setState(TURNING);
        return;
    }

    if (escapeStep == 0 && controller.isIdle()) {
        controller.moveDistance(80.0, false);
        escapeStep = 1;
    }
    else if (escapeStep == 1 && controller.isIdle()) {

        if (FL > FR) {
            controller.requestTurn(-45);
        } else controller.requestTurn(45);

        escapeStep = 2;
    }
    else if (escapeStep == 2 && controller.isIdle()) {
        setState(DRIVE);
        escapeStep = 0;
    }
}
void Exploration::handleCelly(){
    if (!controller.isIdle()) return;

    if (goalDirection== NORTH && !goal1Celly) {

        controller.requestTurn(1080);
        goal1Celly = true;
        
        setState(CELLY_WAIT);
        return;
    }

    if (goalDirection == SOUTH && !goal2Celly) {
        // controller.requestTurn(720);
        controller.cancel();
        goal2Celly = true;
        setState(IDLE);
        return;
    }
}
void Exploration::handleCellyWait(){
    Serial.println("CELLY_WAIT");

    if (!controller.isIdle()) return;

    goalDirection = SOUTH;
    goalY = 150;

    setState(TURN_TO_GOAL);
}




// ============== STATE MACHINE MANAGEMENT =====================


void Exploration::setState(ExplorationStates next){
    if (explorationState == next) return;
    prevState = explorationState;
    explorationState = next;
}


// =================== MAPPING ====================================


bool Exploration::toCell(float x_mm, float y_mm, int &cx, int &cy) {
    // update current x and y 
    cx = (int)floorf(x_mm / CELL_MM); // column 0-13
    cy = (int)floorf(y_mm / CELL_MM); // row 0-19

    if (cx < 0 || cx >= MAP_COLUMNS || cy < 0 || cy >= MAP_ROWS) return false; // out of bounds return false
    return true;
}

void Exploration::markVisited(float x, float y) {

    float dx = x - lastMarkX;
    float dy = y - lastMarkY;

    // mark after 50mm
    if (sqrtf(dx*dx + dy*dy) < 50.0f) return;

    int cx, cy;
    if (toCell(x, y, cx, cy)) {
        if (map[cx][cy] != 0) return; // if marked as anything else then return
        map[cx][cy] = 1; // 1 is visited
    }

    lastMarkX = x;
    lastMarkY = y;
}

void Exploration::markObstacles(float x, float y, float theta) {
    // uses sensors to create occupancy grid but not used, too noisy

    float F = sensors.getFrontDist();
    
    if (F < 100.0f && F > 20.0f) {
        float obstX = x + F * cosf(theta) * 10;
        float obstY = y + F * sinf(theta) * 10;

        int cx, cy;
        if (toCell(obstX, obstY, cx, cy)) {
            map[cx][cy] = 2; // 2 is obstacle
        }
    }
}

// tried logic for going back to other squares but created weird behaviour
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






// ====================== NAV LOGIC ==============================

float Exploration::chooseHeadingFromFrontArray(float FL, float F, float FR, float R, float IR_L, float IR_R) {

    const float FL_CLOSE = 40.0f;
    const float F_CLOSE  = 40.0f;
    const float FR_CLOSE = 40.0f;
    const float OBSTACLE_PENALTY = -120.0f;
    float theta = controller.getTheta();
    
    //  create 3 turn options, -40, 0, +40
    float turnAngle = 40.0f * PI/180.0f;
    float headingL = wrapPi(theta + turnAngle);
    float headingF = theta;
    float headingR = wrapPi(theta - turnAngle);

    static float lastHeading = 0.0f;
    const float HYSTERESIS_BONUS = 30.0f; // try to smoothen out

    // emergency steer away (needs a bit more tuning i think)
    const float SIDE_PANIC = 20.0f;
    if (FL < SIDE_PANIC) return headingR;
    if (FR < SIDE_PANIC) return headingL;

    // goal alignment scores
    float goalWeight = 80.0f;
    float goalL = cosf(headingL - goalDirection);
    float goalF = cosf(headingF - goalDirection);
    float goalR = cosf(headingR - goalDirection);

    // attempt at punishing if facing south, didnt work too well
    // float southPunishL = goalL < -0.5 ? OBSTACLE_PENALTY : 0.0f;
    // float southPunishF = goalF < -0.5 ? OBSTACLE_PENALTY : 0.0f;
    // float southPunishR = goalF < -0.5 ? OBSTACLE_PENALTY : 0.0f;

    // attempt at weighting more positive sides harder, introduces too much complexity
    // clearL = FL < FL_CLOSE ? OBSTACLE_PENALTY : (  (goalL > 0) ? (FL * 1.4) : FL  );
    // clearF = F  < F_CLOSE  ? OBSTACLE_PENALTY : (  (goalF > 0) ? (F  * 1.4) : F   );
    // clearR = FR < FR_CLOSE ? OBSTACLE_PENALTY : (  (goalR > 0) ? (FR * 1.4) : FR  );

    // clearance scores inc obstacle pens
    float clearL = FL < FL_CLOSE ? OBSTACLE_PENALTY : FL;
    float clearF = F  < F_CLOSE  ? OBSTACLE_PENALTY : F;
    float clearR = FR < FR_CLOSE ? OBSTACLE_PENALTY : FR;

    // capping values to not control output
    clearL = min(clearL, 160.0f);
    clearF = min(clearF, 160.0f);
    clearR = min(clearR, 160.0f);

    // steer away if IR too close to keep away from walls
    float steerL = IR_R < 15.0f ? 200.0f : 0.0f;
    float steerR = IR_L < 15.0f ? 200.0f : 0.0f;

    // if side walls are lcose then dont keep going forward
    const float SIDE_NEAR = 110.0f;
    float minSide = min(FL, FR);
    if (minSide < SIDE_NEAR) {
        // stronger penalty the closer you are to a side wall
        clearF -= (SIDE_NEAR - minSide);   // cheap, linear, effective
    }

    // calc final scores
    float sFL = clearL + goalL * goalWeight + steerL;
    float sF  = clearF + goalF * goalWeight;
    float sFR = clearR + goalR * goalWeight + steerR;

    // bonus forward if in open spaces
    if (FL > 100.0f && FR > 100.0f && FL > 100.0f) {
        sF += 80.0f;
    }

    // punishment if really south 
    // if (goalL < -0.8) sFL -= 50.0f;
    // if (goalF < -0.8) sF  -= 50.0f;
    // if (goalR < -0.8) sFR -= 50.0f;

    // prefer same direction hysteresis
    float deltaL = fabsf(wrapPi(headingL - lastHeading));
    float deltaF = fabsf(wrapPi(headingF - lastHeading));
    float deltaR = fabsf(wrapPi(headingR - lastHeading));
    
    if (deltaL < 0.1f) sFL += HYSTERESIS_BONUS;  // Was going left
    if (deltaF < 0.1f) sF  += HYSTERESIS_BONUS;  // Was going straight
    if (deltaR < 0.1f) sFR += HYSTERESIS_BONUS;  // Was going right

    // return best heading choice
    float headingChoice;
    if (sFL >= sF && sFL >= sFR) {
        headingChoice = headingL;
    }
    else if (sFR >= sF && sFR >= sFL) {
        headingChoice = headingR;
    } else {
        headingChoice = headingF;
    }

    lastHeading = headingChoice;
    return headingChoice;
}


int Exploration::pickTurnDeg(float FL, float F, float FR) {

    bool hitL = FL < COLLISION_FL;
    bool hitC = F  < COLLISION;
    bool hitR = FR < COLLISION_FR;

    // completely blocked
    if (hitL && hitC && hitR) return  90;

    // blocked combos
    if (hitC && hitL)         return -90; // turn right
    if (hitC && hitR)         return  90; // turn left
    if (!hitC && hitL)        return -45; // turn a little bit
    if (!hitC && hitR)        return  45;
    
    if (hitC) {
        const float MIN_DIFF = 8.0f;
        if (FR > FL + MIN_DIFF) return -90;  // right clearly better
        if (FL > FR + MIN_DIFF) return  90;   // left clearly better
        
        // if similar, pick bigger one
        return (FR > FL) ? -90 : 90;
    }
    return 0; // dont need to turn ( will never get here but is safe)
}