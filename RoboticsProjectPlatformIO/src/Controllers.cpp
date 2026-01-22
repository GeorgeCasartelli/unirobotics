#include <Controllers.h>
#include <Arduino.h>
#include <mbed.h>
#include <Motors.h>
#include <Gyro.h>

Controllers::Controllers(Motors &motor, Gyro &gyro) 
    : motors(motor), gyro(gyro)  // initialize the member with the external Motors object
{
    // Initialize controller parameters
    resetPose();
    prevDistA = 0.0f;
    prevDistB = 0.0f;
    distanceTarget = 0.0f;
    startDistanceA = 0.0f;
    startDistanceB = 0.0f;
    movingForward = true;

    turnTarget =  0.0f;
    pendingTurnTarget = 0.0f;
    turnTargetDeg = 0.0f;
    startAngle = 0.0f;

    controllerState = IDLE;

    obstacleDetected = false;
    rightDistanceAvg = 0.0f;

    // targetWallDistance = 12.0f;
    currentWallDistance = 0.0f;

    wallHistoryIndex = 0;
    for (int i = 0; i < 3; i++) {
      wallDistanceHistory[i] = 0.0f;

    }
    filteredWallDistance = 0.0f;
}

void Controllers::moveDistance(float target, bool forward) {
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

  void Controllers::requestTurn(float angle) {
    pendingTurnTarget = angle;
    motors.setDirection(angle > 0 ? 0 : 1, angle > 0 ? 1 : 0);
    setState(TURN_PREP);
  }

  void Controllers::requestWallAlign() {
    setState(ALIGN_TO_WALL);
  }

  void Controllers::cancel() {
    motors.stop();
    gotoStep = NONE;
    setState(IDLE);
  }

  bool Controllers::isIdle() {
    return controllerState == IDLE;
  }

  void Controllers::setState(STATES next) {
    if (controllerState == next) return;
    prevState = controllerState;
    controllerState = next;
    onEnterState();
  }



  void Controllers::goToPose(float xg, float yg, float thetag) {
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

  void Controllers::resetPose() {
    x = 0.0f;
    y = 0.0f;
    theta = 0.0f; // facing +x
  }

  void Controllers::startPose() {
    x = 50.0f;
    y = 50.0f;
    theta = 0.0f;
  }
  
  float Controllers::getClosestCardinal(float theta) {
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

  void Controllers::updatePose() {
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

  void Controllers::requestTurnToHeading(float target) {
    float error = wrapPi(target - getTheta());
    requestTurn(error * (180.0f/PI));
  }


  void Controllers::startWallFollowing(float targetDist, float speed) {
    targetWallDistance = targetDist;
    baseWallSpeed = speed;
    filteredWallDistance = currentWallDistance;
    motors.setDirection(1,1);
    leftSign = 1;
    rightSign = 1;
    distLocked = false;
    setState(WALL_FOLLOWING);
  }

  void Controllers::setWallDistance(float distance) {
    // currentWallDistance = distance;
    filteredWallDistance = 0.8f * filteredWallDistance + 0.2f * distance;
  }

  void Controllers::recalibrate() {

  }

  void Controllers::followWall() {
    // float error = currentWallDistance - targetWallDistance;
    float distCm = 0.5f * (rightFrontIR + rightRearIR);
    // float error = filteredWallDistance - targetWallDistance;
    float distError = distCm - targetWallDistance;
    float angleError = (rightFrontIR - rightRearIR);
    float angleRad = atan2f(rightFrontIR - rightRearIR, 4.2);
    float correction;
    bool wallPresent = rightUS < 20.0f;
    bool edgePeek = (rightFrontIR > rightRearIR + 4.0);

    correction = Kp_angle * angleError + Kp_dist * distError;

    correction = constrain(correction, -0.12f, 0.12f);

    float leftSpeed = baseWallSpeed + correction;
    float rightSpeed = baseWallSpeed - correction;

    leftSpeed = constrain(leftSpeed, 0.25f, baseWallSpeed+0.15f);
    rightSpeed = constrain(rightSpeed, 0.25f, baseWallSpeed+0.15f);

    // Serial.println((String)"WF: rightFront = " + rightFrontIR + ", rightRear = " + rightRearIR + ", \r\n distError = "+ distError+", angleError = " + angleError + ", correction  = " + correction  + "\r\n L="+leftSpeed+", R="+rightSpeed);
    motors.setTargetSpeeds(leftSpeed, rightSpeed);
  }

  void Controllers::moveContinuous(bool forward, float speed) {
    controllerState = IDLE;
  }

  void Controllers::runGoToStep() {
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

  void Controllers::onEnterState() {
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
        if (turnLeft) { leftSign = - 1; rightSign = 1; }
        else          { leftSign = 1;   rightSign = -1;}

        motors.setTargetSpeeds(0.2f, 0.2f);
        alignCount = 0;
        // Serial.println((String)"rgightFront: " + rightFrontIR + ", rightRear: " + rightRearIR + ", offset: "+offset+", turnLeft: "+turnLeft+"\r\nleftDir: "+ leftDir + ", rightDit: " + rightDir);
        break;
      }
    }
  }

  void Controllers::align() {
    setState(ALIGN_TO_WALL);
  }

  void Controllers::driveHeading(float targetHeadingRad, float baseSpeed){
    if (motors.getCurrentLeftDir() != 1 || motors.getCurrentRightDir() != 1) {
      motors.setDirection(1, 1);
      leftSign = 1;
      rightSign = 1;
      movingForward = true;
    }
    
    float error = wrapPi(targetHeadingRad - getTheta());
    float steer = K_heading * error;

    steer = constrain(steer, -maxSteer, maxSteer);

    float left = baseSpeed - steer;
    float right = baseSpeed + steer;

    left = constrain(left, minPWM, 1.0f);
    right = constrain(right, minPWM, 1.0f);

    motors.setTargetSpeeds(left, right);

    setState(HEADING_DRIVE);

  }

  void Controllers::update(){

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
        float error = distanceTarget - ds;

        if (fabs(error) < tolerance) {
          motors.stop();
          setState(IDLE);
          break;
        }

        if (movingForward && error < 0.0f) {
          motors.stop();
          setState(IDLE);
          break;
        }
        if (!movingForward && error > 0.0f) {
          motors.stop();
          setState(IDLE);
          break;
        }

        // p control
        float speed = fabs(Kp * error);
        
        // limit speeds
        if (speed > 0.0f && speed < minPWM) {
          speed = minPWM;
        }

        speed = constrain(speed, 0.0f, 1.0f);

        motors.setTargetSpeeds(speed, speed);

        break;
      };

      case TURN_PREP: {

        if (!motors.isStopped()) { break; }
        if (motors.isChangingDir()) { break; }

        if (pendingTurnTarget > 0) { leftSign = -1; rightSign = 1;}
        else                       { leftSign = 1;  rightSign = -1;}    

        startDistanceA = motors.getDistanceA();
        startDistanceB = motors.getDistanceB();

        turnTargetDeg = pendingTurnTarget;
        turnTarget = turnTargetDeg * (PI/180.0f);

        prevTurnError = turnTarget;
        turnErrorInit = true;
        setState(TURNING);
        motors.setTargetSpeeds(0.4f, 0.4f);
        // turnDegrees(pendingTurnTarget); // once in the if statement, state sets to TURNING
        break;
      }
    
      case TURNING: {
        float error = turnTarget - dtheta;
        float fabsError = fabs(error);

        if (fabsError < turnTolerance) {
          // Serial.println("WITHIN TOLERANCE. STOPPING");
          motors.stop();
          setState(IDLE);
          break;
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
            break;
          }
        }
        
        float speed;

        speed = Kp_turn * fabsError;
        

        speed =constrain(speed, minPWM, maxTurnSpeed);

        // Serial.println((String)"Error: " + error);
        motors.setTargetSpeeds(speed, speed);
        // Serial.println
        break;
      };

      case WALL_FOLLOWING: {
        

        followWall();

        break;
      }

      case ALIGN_TO_WALL: {

        float offset = rightFrontIR - rightRearIR;
        alignCount++;

        Serial.println((String)"distFront: "+ rightFrontIR + ", distRear: "+rightRearIR);

        if (fabs(offset) < 0.05f) {
            motors.stop();
            theta = getClosestCardinal(getTheta());
            setState(IDLE);
            Serial.println((String)"ALIGNED! Final distFront: "+ rightFrontIR + ", distRear: "+rightRearIR);
            break;
        }

        if (alignCount > 100) {
            motors.stop();
            theta = getClosestCardinal(getTheta());
            setState(IDLE);
            Serial.println((String)"ALIGN TIMEOUT - good enough. Final distFront: "+ rightFrontIR + ", distRear: "+rightRearIR);
            break;
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
        motors.setTargetSpeeds(0.2f, 0.2f);
        
        Serial.println((String)"Aligning... offset = " + offset + ", count = " + alignCount);
        break;

      }

      case HEADING_DRIVE: {

        break;
      }
      
      case IDLE: {

        break;
      }
  }
      
    
    motors.update();
  }


  void reset() {

  }
