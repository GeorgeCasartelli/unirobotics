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

    turnTarget = 40.0f *(3.14159f / 180.0f);
    turnTargetDeg = 40.0f;
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

  void Controllers::turnDegrees(float angle) {
    // Serial.println("Turn degrees called");

    gyro.reset();

    startDistanceA = motors.getDistanceA();
    startDistanceB = motors.getDistanceB();

    turnTargetDeg = angle;
    turnTarget = angle * (3.14159f / 180.0f);
    // distanceTarget = (trackWidth / 2.0f) * fabs(angleRad); // arclength, but set angle to positive
    // movingForward = (angle > 0);

    controllerState = TURNING;
    
    // Serial.println((String)"Turn degrees variables: \r\n  startDistanceA and B: " + startDistanceA + "   " + startDistanceB + "\r\n  DistanceTarget: " + distanceTarget);

    if (angle > 0) {
      motors.setDirection(0,1);
      leftSign = -1;
      rightSign = 1;
    } else {
      motors.setDirection(1,0);
      leftSign = 1;
      rightSign = -1;
    }
    motors.setTargetSpeeds(0.5f, 0.5f);
  }

  const char* Controllers::stateToString(STATES s) {
      switch (s) {
          case IDLE: return "IDLE";
          case MOVING: return "MOVING";
          case TURNING: return "TURNING";
          case GOTO: return "GOTO";
          default: return "UNKNOWN";
      };
  }

  bool Controllers::isIdle() {
    return controllerState == IDLE;
  }

  void Controllers::transitionTo(STATES next) {
    if (controllerState == next) return;
    prevState = controllerState;
    controllerState = next;
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
    Serial.println((String)"goToPose Received: \r\n     dx: " + dx + "\r\n     dy: " + dy + "\r\n     ds: "+ goToDistance + "\r\n     heading: "+ goToHeading);
    Serial.println((String)"Target pose: { x = " + xg + ", y = " + yg + ", thetag = " + thetag + "}");
  }

  void Controllers::resetPose() {
    x = 0.0f;
    y = 0.0f;
    theta = 0.0f; // facing +x
  }
  

  void Controllers::updatePose() {
    // update local pose of the robot
    float distA = motors.getDistanceA();
    float distB = motors.getDistanceB();

    float dA = distA - prevDistA;
    float dB = distB - prevDistB;

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


    // Serial.println((String)"POSE: [\r\n       x: "+x+", \r\n       y: " + y + ", \r\n   theta: "+ theta*(180/PI) + "\r\n]");
  }

  void Controllers::runGoToStep() {
    switch (gotoStep) {
        case TURN1: {
          float err = goToHeading - getTheta();
          // turnDegrees(err * (180.0f / PI));
          Serial.println((String)"TURN1: Need to turn: " + err * (180.0f/PI));
          turnDegrees(err*(180.0f/PI));
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
          transitionTo(IDLE);
          break;
        }

        if (movingForward && error < 0.0f) {
          motors.stop();
          transitionTo(IDLE);
          break;
        }
        if (!movingForward && error > 0.0f) {
          motors.stop();
          transitionTo(IDLE);
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
    
      case TURNING: {
        float error = turnTarget - dtheta;

        if (fabs(error) < turnTolerance) {
          motors.stop();
          transitionTo(IDLE);
          break;
        }

        float speed = fabs(Kp_turn * error);

        if (speed > 0.0f && speed < minPWM) speed = minPWM; // bottom limit
        speed = constrain(speed, 0.0f, maxTurnSpeed);

        motors.setTargetSpeeds(speed, speed);
        break;
      };
      
      case IDLE: {

        break;
      }
  }
      

    motors.update();
  }


  void reset() {

  }
