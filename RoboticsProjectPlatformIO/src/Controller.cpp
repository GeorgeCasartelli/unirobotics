#include <Controller.h>
#include <Arduino.h>
#include <mbed.h>
#include <Motors.h>
#include <Gyro.h>

Controller::Controller(Motors &motor, Gyro &gyro) 
    : motors(motor), gyro(gyro)  
{
    // vars initialised in header
    resetPose();
}


// ========== motion control ==============================

void Controller::moveDistance(float target, bool forward) {
    // Serial.println("Turn degrees called");
    // set start vals to distance of motors
    startDistanceA = motors.getDistanceA();
    startDistanceB = motors.getDistanceB();
    int sign = forward ? 1 : -1;
    distanceTarget = target * sign;

    movingForward = forward;
    
    controllerState = MOVING;

    if (movingForward) {  
      motors.setDirection(1, 1);
      leftSign = 1;
      rightSign = 1;
    }
    else { 
      motors.setDirection(0, 0);
      leftSign = -1;
      rightSign = -1;
    }
    // Serial.println("Move distance called and target speed set");
    motors.setTargetSpeeds(0.7f, 0.7f);
  }

  void Controller::requestTurn(float angle) {
    pendingTurnTarget = angle;
    motors.setDirection(angle > 0 ? 0 : 1, angle > 0 ? 1 : 0);
    setState(TURN_PREP);
  }

   void Controller::requestTurnToHeading(float target) {
    float error = wrapPi(target - getTheta());
    requestTurn(error * (180.0f/PI));
  }

  void Controller::driveHeading(float targetHeadingRad, float baseSpeed){
    // update values for state machine to use
    targetHeading = targetHeadingRad;
    headingDriveSpeed = baseSpeed;

    if (controllerState != HEADING_DRIVE) {
      if (motors.getCurrentLeftDir() != 1 || motors.getCurrentRightDir() != 1) {
        motors.setDirection(1, 1);
        leftSign = 1;
        rightSign = 1;
        movingForward = true;
      }
      setState(HEADING_DRIVE);
    }
  }

  void Controller::cancel() {
    motors.stop();
    gotoStep = NONE;
    setState(IDLE);
  }

  // ====================== wall follow ==================

  void Controller::startWallFollowing(float targetDist, float speed) {
    targetWallDistance = targetDist;
    baseWallSpeed = speed;
    filteredWallDistance = currentWallDistance;
    motors.setDirection(1,1);
    leftSign = 1;
    rightSign = 1;
    distLocked = false;
    setState(WALL_FOLLOWING);
  }

  void Controller::requestWallAlign() {
    setState(ALIGN_TO_WALL);
  }

  // ================ navigation ===========================
  void Controller::goToPose(float xg, float yg, float thetag) {
    goalX = xg;
    goalY = yg;
    goalTheta = thetag;

    float dx = goalX - getX();
    float dy = goalY - getY();

    goToDistance = sqrt(dx*dx + dy*dy); // ds
    goToHeading = atan2(dy, dx); // rads

    gotoStep = TURN1;
    // Serial.println((String)"goToPose Received: \r\n     dx: " + dx + "\r\n     dy: " + dy + "\r\n     ds: "+ goToDistance + "\r\n     heading: "+ goToHeading);
    // Serial.println((String)"Target pose: { x = " + xg + ", y = " + yg + ", thetag = " + thetag + "}");
  }

  void Controller::startPose() {
    x = 50.0f;
    y = 50.0f;
    theta = 0.0f;
  }

  // state queries ====================================

  bool Controller::isIdle() {
    return controllerState == IDLE;
  }

  // ==================== update loop ========================

    void Controller::update(){

      updatePose();
      float dl = leftSign * (motors.getDistanceA() - startDistanceA); // left wheel change
      float dr = rightSign * (motors.getDistanceB() - startDistanceB); // right wheel change
      float ds = (dl + dr) / 2; // displacement change
      float dtheta = (dr - dl) / trackWidth; // angle change

      
      if (stateToString(controllerState) != "IDLE") {
        if (printStates) Serial.println((String)"Controller State: " + stateToString(controllerState));
      }
      
      if (gotoStep != NONE && controllerState == IDLE) {
        runGoToStep();
      }

      switch(controllerState) {

        case MOVING: {
          handleMoving(ds);

          break;
        };

        case TURN_PREP: {

          handleTurnPrep();
          break;
        }
      
        case TURNING: {
          handleTurning(dtheta);
          break;
        };

        case WALL_FOLLOWING: {
          handleWallFollowing();
          break;
        }

        case ALIGN_TO_WALL: {
          handleAlignToWall();
          break;
        }

        case HEADING_DRIVE: {
          handleHeadingDrive();
          break;
        }
        
        case IDLE: {

          break;
        }
    }
        
      
      motors.update();
  }

  // ==================== state handlers ==================================

  void Controller::handleMoving(float ds) {
    float error = distanceTarget - ds;
    if (fabs(error) < DIST_TOLERANCE) {
      motors.stop();
      setState(IDLE);
      return;
    }

    if (movingForward && error < 0.0f) {
      motors.stop();
      setState(IDLE);
      return;
    }
    if (!movingForward && error > 0.0f) {
      motors.stop();
      setState(IDLE);
      return;
    }

    // p control
    float speed = fabs(KP * error);
    
    // limit speeds
    if (speed > 0.0f && speed < minPWM) {
      speed = minPWM;
    }

    speed = constrain(speed, 0.0f, 1.0f);
    motors.setTargetSpeeds(speed, speed);
  }

  void Controller::handleTurnPrep() {
    if (!motors.isStopped() || motors.isChangingDir()) return;

    if (pendingTurnTarget > 0) { 
      leftSign = -1; 
      rightSign = 1;
    } else { 
      leftSign = 1;  
      rightSign = -1;
    }    

    startDistanceA = motors.getDistanceA();
    startDistanceB = motors.getDistanceB();

    turnTargetDeg = pendingTurnTarget;
    turnTarget = turnTargetDeg * (PI/180.0f);

    prevTurnError = turnTarget;
    turnErrorInit = true;

    setState(TURNING);
    motors.setTargetSpeeds(0.4f, 0.4f);
  }

  void Controller::handleTurning(float dtheta) {
    float error = turnTarget - dtheta;
    float absError = fabs(error);

    if (absError < TURN_TOLERANCE) {
      // Serial.println("WITHIN TOLERANCE. STOPPING");
      motors.stop();
      setState(IDLE);
      return;
    }

    // overshoot stop: if we crossed through zero, stop
    if (turnErrorInit) {
      bool signFlip = (prevTurnError > 0.0f && error < 0.0f) ||
                      (prevTurnError < 0.0f && error > 0.0f);
      prevTurnError = error;

      if (signFlip) {
        motors.stop();
        setState(IDLE);
        turnErrorInit = false;
        return;
      }
    }
    
    float speed;

    speed = KP_TURN * absError;
    speed =constrain(speed, minPWM, MAX_TURN_SPEED);
    motors.setTargetSpeeds(speed, speed);
  }

  void Controller::handleWallFollowing() {
    followWall();
  }

  void Controller::handleAlignToWall() {
    float offset = rightFrontIR - rightRearIR;
    alignCount++;

    Serial.println((String)"distFront: "+ rightFrontIR + ", distRear: "+rightRearIR);

    if (fabs(offset) < 0.05f) {
        motors.stop();
        theta = getClosestCardinal(getTheta());
        setState(IDLE);
        Serial.println((String)"ALIGNED! Final distFront: "+ rightFrontIR + ", distRear: "+rightRearIR);
        return;
    }

    if (alignCount > ALIGN_TIMEOUT) {
        motors.stop();
        theta = getClosestCardinal(getTheta());
        setState(IDLE);
        Serial.println((String)"ALIGN TIMEOUT - good enough. Final distFront: "+ rightFrontIR + ", distRear: "+rightRearIR);
        return;
    }


    bool turnLeft = (offset < 0.0f);
    uint8_t leftDir = turnLeft ? 0 : 1;
    uint8_t rightDir = turnLeft ? 1 : 0;

    if (!motors.isChangingDir()) {
      if (motors.getCurrentLeftDir() != leftDir || motors.getCurrentRightDir() != rightDir) {
        motors.setDirection(leftDir, rightDir);
        leftSign = turnLeft ? -1 : 1;
        rightSign = turnLeft ? 1 : -1;
      }
    }
    // Just set speed, direction was already set in onEnterState
    motors.setTargetSpeeds(ALIGN_SPEED, ALIGN_SPEED);
    
    Serial.println((String)"Aligning... offset = " + offset + ", count = " + alignCount);
  }

  void Controller::handleHeadingDrive() {
    float error = wrapPi(targetHeading - getTheta());
    float steer = K_HEADING * error;

    steer = constrain(steer, -MAX_STEER, MAX_STEER);

    float left = headingDriveSpeed - steer;
    float right = headingDriveSpeed + steer;

    left = constrain(left, minPWM, 1.0f);
    right = constrain(right, minPWM, 1.0f);

    motors.setTargetSpeeds(left, right);
  }

  // =================================================================
  // =================== state machine tranmsitons ==============
  // ===============================================================


  void Controller::setState(STATES next) {
    if (controllerState == next) return;
    prevState = controllerState;
    controllerState = next;
    onEnterState();
  }

  void Controller::onEnterState() {
    switch(controllerState) {
      case ALIGN_TO_WALL: {
        float distance = (rightFrontIR + rightRearIR) * 0.5;
        if (distance > 20.0f) { 
          // Serial.println("Too far, setting back to wall follow"); 
          setState(WALL_FOLLOWING); 
          break ;
        }
        float offset = rightFrontIR - rightRearIR;
        bool turnLeft = (offset < 0.0f);
        int leftDir = turnLeft ? 0: 1;
        int rightDir = turnLeft ? 1 : 0;

        motors.setDirection(leftDir, rightDir);
        if (turnLeft) { 
          leftSign = - 1; 
          rightSign = 1; 
        }
        else { 
          leftSign = 1;   
          rightSign = -1;
        }

        motors.setTargetSpeeds(ALIGN_SPEED, ALIGN_SPEED);
        alignCount = 0;
        // Serial.println((String)"rgightFront: " + rightFrontIR + ", rightRear: " + rightRearIR + ", offset: "+offset+", turnLeft: "+turnLeft+"\r\nleftDir: "+ leftDir + ", rightDit: " + rightDir);
        break;
      }

      default:
      break;  
    }
  }

  void Controller::runGoToStep() {
    switch (gotoStep) {
      case TURN1: {
        float err = goToHeading - getTheta();
        // turnDegrees(err * (180.0f / PI));
        Serial.println((String)"TURN1: Need to turn: " + err * (180.0f/PI));
        requestTurn(err*(180.0f/PI));
        gotoStep = DRIVE;
        break;
      }
    
      case DRIVE: {
        moveDistance(goToDistance, true);

        Serial.println((String)"DRIVE: Need to move: " + goToDistance);
        gotoStep = DONE;
        break;
      }

      case DONE: {
        Serial.println((String)"DONE MOVEMENT!!! POSE IS NOW: { " + getX() + ", " + getY() + ", " + getTheta() + " }" );
        gotoStep = NONE;
      }

      default: 
      case NONE:
        break;
    }  
  }


  // ====================== pose ====================== 
  void Controller::resetPose() {
    x = 0.0f;
    y = 0.0f;
    theta = 0.0f; // facing +x
  }

  void Controller::updatePose() {
    // update local pose of the robot
    float distA = motors.getDistanceA();
    float distB = motors.getDistanceB();

    float dB = distA - prevDistA;
    float dA = distB - prevDistB;

    prevDistA = distA;
    prevDistB = distB;

    // apply +/- signs
    float dl = dA * leftSign;
    float dr = dB * rightSign;

    float ds = 0.5 * (dl + dr);
    float dtheta = (dr - dl) / trackWidth;

    theta += dtheta;
    //keep theta bound
    while (theta > PI) theta -= 2*PI;
    while (theta < -PI) theta += 2*PI;

    //calc x/y value from distance and theta
    x += ds * cosf(theta);
    y += ds * sinf(theta);


    // Serial.println((String)"POSE: [       x: "+x+",        y: " + y + ",   theta: "+ theta*(180/PI) + "    ]");
  }

  void Controller::setWallDistance(float distance) {
    // currentWallDistance = distance;
    filteredWallDistance = WALL_FILTER * filteredWallDistance + 0.2f * distance;
  }


  void Controller::followWall() {
    // float error = currentWallDistance - targetWallDistance;
    float distCm = 0.5f * (rightFrontIR + rightRearIR);
    // float error = filteredWallDistance - targetWallDistance;
    float distError = distCm - targetWallDistance;
    float angleError = (rightFrontIR - rightRearIR);
    float correction;
    bool wallPresent = rightUS < WALL_PRESENT;
    bool edgePeek = (rightFrontIR > rightRearIR + 4.0);

    correction = KP_WALL_ANGLE * angleError + KP_DIST * distError;

    correction = constrain(correction, -WALL_CORRECTION, WALL_CORRECTION);

    float leftSpeed = baseWallSpeed + correction;
    float rightSpeed = baseWallSpeed - correction;

    leftSpeed = constrain(leftSpeed, WALL_SPEED_MIN, baseWallSpeed+0.15f);
    rightSpeed = constrain(rightSpeed, WALL_SPEED_MIN, baseWallSpeed+0.15f);

    // Serial.println((String)"WF: rightFront = " + rightFrontIR + ", rightRear = " + rightRearIR + ", \r\n distError = "+ distError+", angleError = " + angleError + ", correction  = " + correction  + "\r\n L="+leftSpeed+", R="+rightSpeed);
    motors.setTargetSpeeds(leftSpeed, rightSpeed);
  }

  void Controller::moveContinuous(bool forward, float speed) {
    controllerState = IDLE;
  }

  
  float Controller::getClosestCardinal(float theta) {
    const float c[4] = { 0.0f, PI/2, PI, -PI/2};
    float best = c[0];
    float bestErr = 1e9;

    for (int i=0; i<4; i++) {
      float e = fabs(wrapPi(theta - c[i]));
      if ( e < bestErr) { bestErr = e; best = c[i]; }
    }
    Serial.println((String)"Closest cardinal: " + best*(180/PI));
    return best; // RADIANS
  }

  void Controller::align() {
    setState(ALIGN_TO_WALL);
  }

  



  void reset() {

  }
