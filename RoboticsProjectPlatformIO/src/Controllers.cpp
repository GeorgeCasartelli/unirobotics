#include <Controllers.h>
#include <Arduino.h>
#include <mbed.h>
#include <Motors.h>

Controllers::Controllers(Motors &motor) 
    : motors(motor)  // initialize the member with the external Motors object
{
    // Initialize controller parameters
    maxTurnSpeed = 0.6f;
    minTurnSpeed = 0.2f;

    distanceTarget = 0.0f;
    startDistanceA = 0.0f;
    startDistanceB = 0.0f;
    movingForward = true;

    turnTarget = 0.0f;
    startAngle = 0.0f;

    controllerState = IDLE;

    obstacleDetected = false;
    rightDistanceAvg = 0.0f;
}

void Controllers::moveDistance(float target, bool forward) {
    Serial.println("Turn degrees called");
    // set start vals to distance of motors
    startDistanceA = motors.getDistanceA();
    startDistanceB = motors.getDistanceB();
    float avg_distance = ((motors.getDistanceA() - startDistanceA) + (motors.getDistanceB() - startDistanceB)) / 2.0f;
    distanceTarget = target + avg_distance;

    movingForward = forward;
    
    controllerState = MOVING;

    if (movingForward) {  
      motors.setDirection(1, 1);
    }
    else { 
      motors.setDirection(0, 0);
    }
    // Serial.println("Move distance called and target speed set");
    motors.setTargetSpeeds(0.7f, 0.7f);
  }

  void Controllers::turnDegrees(float angle) {
    Serial.println("Turn degrees called");

    startDistanceA = motors.getDistanceA();
    startDistanceB = motors.getDistanceB();

    float angleRad = angle * (3.14159f / 180.0f);
    distanceTarget = (trackWidth / 2.0f) * fabs(angleRad); // arclength, but set angle to positive
    movingForward = (angle > 0);

    controllerState = TURNING;
    
    Serial.println((String)"Turn degrees variables: \r\n  startDistanceA and B: " + startDistanceA + "   " + startDistanceB + "\r\n  DistanceTarget: " + distanceTarget);

    if (angle > 0) {
      motors.setDirection(1,0);
    } else {
      motors.setDirection(0,1);
    }
    motors.setTargetSpeeds(0.5f, 0.5f);
  }

  float Controllers::calculateTrapezoidalSpeed(float traveled, float totalDistance, float maxSpeed) {
    // float speed;

    float accelDist = totalDistance * 0.25f;
    float decelDist = totalDistance * 0.25f;

    float error = totalDistance - traveled;

    if (traveled < accelDist) {
          float progress = traveled / accelDist;
          float smoothProgress = (1.0f - cos(progress * 3.14159f)) / 2.0f;
          return minTurnSpeed + (maxSpeed - minTurnSpeed) * progress;
        }

        else if (error < decelDist) {
          float progress = error / decelDist;
          float smoothProgress = (1.0f - cos(progress * 3.14159f)) / 2.0f;
          return minTurnSpeed + (maxSpeed - minTurnSpeed) * progress;
          // speed = minTurnSpeed + (maxSpeed - minTurnSpeed) * progress;
        }

        else {
          return maxSpeed;
        }
  }

    const char* Controllers::stateToString(STATES s) {
        switch (s) {
            case IDLE: return "IDLE";
            case MOVING: return "MOVING";
            case TURNING: return "TURNING";
            case CONTINUOUS: return "CONTINUOUS";
            default: return "UNKNOWN";
        };
    }

  bool Controllers::isIdle() {
    return controllerState == IDLE;
  }

  void Controllers::update(){
    
    if (stateToString(controllerState) != "IDLE") {
      if (printStates) Serial.println((String)"Controller State: " + stateToString(controllerState));
    }
    
    switch(controllerState) {

      case MOVING: {
        Serial.println("MOVING");
        float traveled = ((motors.getDistanceA() - startDistanceA) + (motors.getDistanceB() - startDistanceB)) / 2.0f;
        float error = distanceTarget - traveled;

        const float maxSpeed =  1.0f;
        // const float accelDist = 40.0f;
        // const float decelDist = 120.0f;

        float speed = calculateTrapezoidalSpeed(traveled, distanceTarget, maxSpeed);

        motors.setTargetSpeeds(speed, speed);

        if (error < 2.0f) {
          motors.stop();
          controllerState = IDLE;
          Serial.println((String)"Done! Final error: " + error);
        }
        // Serial.println((String)"Err=" + error + " Spd=" + speed + " Trav=" + traveled);
    
      } break;
    
      case TURNING: {
        // Serial.println("TURNING");

        float traveled = ((motors.getDistanceA() - startDistanceA) + (motors.getDistanceB() - startDistanceB)) / 2.0f;
        float error = distanceTarget - traveled;

        if (error <= 0.0f) {
          motors.stop();
          controllerState = IDLE;
          Serial.println((String)"Turn complete! Error: "+error);
          // while(true) {};
          break;
        }

        const float maxSpeed =  0.6f;
        // const float accelDist = 10.0f;
        // const float decelDist = 40.0f;

        float speed = calculateTrapezoidalSpeed(traveled, distanceTarget, maxSpeed);

        motors.setTargetSpeeds(speed, speed);

       
        Serial.println(
            (String)"Turning... Err=" + error +
            " Spd=" + speed +
            " Trav=" + traveled +
            " Target=" + distanceTarget
        );

        break;
      };
      
      case CONTINUOUS: {
        // motors.setTargetSpeeds(0.7f, 0.7f);
        // // Serial.println("CONTINUOUS");
        // if (obstacleDetected) {
        //   Serial.println("Obstacle detectered");
        //   motors.stop();
        //   controllerState = IDLE;
        //   obstacleDetected = false;
        // }
        float wallError = TARGET_WALL_DIST - rightDistanceAvg;
        wallFollow(wallError, 0.8f, 0.00005f);
        break;
      }


      case IDLE: {
        // Serial.println("IDLE");
        motors.stop();
        break;
      }
    }
    motors.update();
  }

  void Controllers::moveContinuous(bool forward, float speed) {
    forward ? motors.setDirection(1,1) : motors.setDirection(0,0);
    motors.setTargetSpeeds(speed, speed);

    Serial.println((String)"Received movement cmd. forward: " + forward + " speed: " + speed);
    controllerState = CONTINUOUS;
  }

  void Controllers::wallFollow(float error, float baseSpeed, float Kp) {
    float correction = Kp * error;
    float leftSpeed = baseSpeed;
    float rightSpeed = baseSpeed;
    // float error = 1500.0f - rightDistanceAvg;
    // correction = constrain(correction, 0.0f, baseSpeed);

    if (error > 0) {
      Serial.println((String)"Tooooo fuckin close. ERROR: " + error);
      leftSpeed = baseSpeed - fabs(correction);
      leftSpeed = constrain(leftSpeed, 0.0f, baseSpeed);
    }
    else if (error < 0) {
      Serial.println((String)"TOOOO FAAAARRRR. ERROR: " + error);
      rightSpeed = baseSpeed - fabs(correction);
      rightSpeed = constrain(rightSpeed, 0.0f, baseSpeed);
    }
    Serial.println((String)"Correction value: " + correction);

    Serial.println((String)"Adjusting speeds to: " + leftSpeed + ", " + rightSpeed);
    motors.setTargetSpeeds(leftSpeed, rightSpeed);
  }
  void Controllers::updateRightWall(float val) {
    rightDistanceAvg = val;
  }

  void Controllers::setObstacleDetected(bool flag) {
    Serial.println("OBSTACLE DETECTED IN CONTROLLER");
    obstacleDetected = flag;
  }

  float Controllers::getAvgDistance() {
    return (motors.getDistanceA() + motors.getDistanceB()) / 2.0f;
  }

  void reset() {

  }
