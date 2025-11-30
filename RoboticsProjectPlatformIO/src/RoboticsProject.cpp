#include <Arduino.h>
#include <mbed.h>
#include <Motors.h>
#include <Infrareds.h>
#include <Ultrasonics.h>
// using namespace mbed;

mbed::DigitalIn JoystickUp(P0_28);
mbed::DigitalIn JoystickDown(P0_29);
mbed::DigitalIn JoystickLeft(P0_30);
mbed::InterruptIn JoystickRight(P0_3);

bool print_data = 0;
bool forwardFlag = 0;

Motors motors;
Infrareds IR;
Ultrasonics US;


namespace Controllers {
  float distanceTarget = 0.0f;
  float startDistanceA = 0.0f;
  float startDistanceB = 0.0f;
  bool movingForward = true;

  float Kp_distance = 0.01f;
  float Kp_angle = 0.02f;

  float turnTarget = 0.0f;
  float startAngle = 0.0f;

  float maxTurnSpeed = 0.7f;
  float minTurnSpeed = 0.2f;

  const float trackWidth = 110.0f;

  bool printStates = true;

  enum STATES {
    IDLE,
    MOVING,
    TURNING
  };

  const char* stateToString(STATES s) {
    switch (s) {
        case IDLE: return "IDLE";
        case MOVING: return "MOVING";
        case TURNING: return "TURNING";
        default: return "UNKNOWN";
    };
}

  STATES controllerState = IDLE;

  void reset();

  void moveDistance(float target, bool forward) {
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
    motors.setTargetSpeed(0.2f);
  }

  void turnDegrees(float angle) {
    Serial.println("Turn degrees called");

    startDistanceA = motors.getDistanceA();
    startDistanceB = motors.getDistanceB();

    float angleRad = angle * (3.14159f / 180.0f);
    distanceTarget = (trackWidth / 2.0f) * fabs(angleRad); // arclength, but set angle to positive
    movingForward = (angle > 0);

    controllerState = TURNING;
    
    Serial.println((String)"Turn degrees variables: \r\n  startDistanceA and B" + startDistanceA + "   " + startDistanceB + "\r\n  DistanceTarget: " + distanceTarget);

    if (angle > 0) {
      motors.setDirection(1,0);
    } else {
      motors.setDirection(0,1);
    }
    motors.setTargetSpeed(0.5f);
  }

  float calculateTrapezoidalSpeed(float traveled, float maxSpeed, float accelDist, float decelDist, float error) {
    float speed;

    if (traveled < accelDist) {
          float progress = traveled / accelDist;
          float smoothProgress = (1.0f - cos(progress * 3.14159f)) / 2.0f;
          speed = minTurnSpeed + (maxSpeed - minTurnSpeed) * smoothProgress;
          // speed = minTurnSpeed + (maxSpeed - minTurnSpeed) * progress;
        }

        else if (error < decelDist) {
          float progress = error / decelDist;
          float smoothProgress = (1.0f - cos(progress * 3.14159f)) / 2.0f;
          speed = minTurnSpeed + (maxSpeed - minTurnSpeed) * smoothProgress;
          // speed = minTurnSpeed + (maxSpeed - minTurnSpeed) * progress;
        }

        else {
          speed = maxSpeed;
        }
      return speed;
  }

  void update(){
    
    if (stateToString(controllerState) != "IDLE") {
      if (printStates) Serial.println((String)"Controller State: " + stateToString(controllerState));
    }
    
    switch(controllerState) {

      case MOVING: {
        Serial.println("MOVING");
        float traveled = ((motors.getDistanceA() - startDistanceA) + (motors.getDistanceB() - startDistanceB)) / 2.0f;
        float error = distanceTarget - traveled;
        // float speed = error * Kp_distance;

        // if (speed > 1.0f) speed = 1.0f;
        // if (speed < 0.0f) speed = 0.0f;


        const float maxSpeed =  1.0f;
        const float accelDist = 40.0f;
        const float decelDist = 120.0f;

        // float speed;
        // if (movingForward) {
        //   motors.setDirection(1,1);
        // } else {
        //   motors.setDirection(0,0);
        // }

        // if (speed < minTurnSpeed) speed = minTurnSpeed;
        // motors.setTargetSpeed(speed);
        
        // if (fabs(error) < 5.0f) {
        //   motors.stop();
        //   controllerState = IDLE;
        // }

        float speed = calculateTrapezoidalSpeed(traveled, maxSpeed, accelDist, decelDist, error);

        motors.setTargetSpeed(speed);

        if (error < 2.0f) {
          motors.stop();
          controllerState = IDLE;
          Serial.println((String)"Done! Final error: " + error);
        }
        Serial.println((String)"Err=" + error + " Spd=" + speed + " Trav=" + traveled);
    
      } break;
    
      case TURNING: {
        Serial.println("TURNING");
        // get differential for turn calc
        // as delta increase -> clockwise,      as delta decreases -> anticlockwise
        // float delta = (motors.getDistanceA() - startDistanceA) - (motors.getDistanceB() - startDistanceB);

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
        const float accelDist = 20.0f;
        const float decelDist = 50.0f;

        float speed = calculateTrapezoidalSpeed(traveled, maxSpeed, accelDist, decelDist, error);
        // float speed = error * Kp_angle;

        // if (speed > maxTurnSpeed) speed = maxTurnSpeed;
        // if (speed < minTurnSpeed) speed = minTurnSpeed;
        
        motors.setTargetSpeed(speed);

       
        Serial.println(
            (String)"Turning... Err=" + error +
            " Spd=" + speed +
            " Trav=" + traveled +
            " Target=" + distanceTarget
        );

        break;
      };

      case IDLE: 
      default:
        // Serial.println("IDLE");
        motors.stop();
        break;
    
    
    }

    // Serial.println((String)"Controller state: " + controllerState);
    motors.update();

    // float distA = fabs(motors.getDistanceA() - startDistanceA);
    // float distB = fabs(motors.getDistanceB() - startDistanceB);
    // float avg = (distA + distB) / 2.0f;

    // if (controllerState == TURNING) {
    //   if (avg >= distanceTarget) {
    //     motors.stop();
    //     controllerState = IDLE; 
    //   }
    //   else {
    //     Serial.println("Not close enough to turn goal!");
    //   }
    // }
    // float avg_traveled = ((motors.getDistanceA() - startDistanceA) + (motors.getDistanceB() - startDistanceB)) / 2.0f;
    // Serial.println((String)"Avg traveled: ?" + avg_traveled + "Distance Target: "+ distanceTarget);
    // if (avg_traveled >= distanceTarget) {
    //   Serial.println("IF stament hit STOPPING motors !");
    //   motors.stop();
    // }
  }

  void reset() {

  }
}


void setup() {
  // get clock speed error for timer
  US.setup();
  motors.setup(JoystickRight);
  
}


void loop() {


  IR.runtime();

  if (print_data) {
    Serial.println((String) "Distance 1: " + IR.distances[0] + "\r\nDistance 2: " + IR.distances[1] + "\r\nDistance 3: " + IR.distances[2] + "\r\nDistance 4: " + IR.distances[3]);
  }

  // US.runtime(0);
  // US.runtime(1);



  // float front = IR::distances[0];
  // float left = US.distances[1];
  // float right = US.distances[0];
  float front = IR.distances[0];

  // else motors.stop();
  if (JoystickUp == 0) {
    Controllers::moveDistance(300.0, true);
  }
  if (JoystickDown == 0) {
    // Controllers::moveDistance(300.0, false);
    Controllers::turnDegrees(90.0);
  }
  if (JoystickLeft == 0) {
    Controllers::turnDegrees(-90.0);
  }

  Controllers::update();
  if (print_data) {
    wait_us(500000);
  }

}
