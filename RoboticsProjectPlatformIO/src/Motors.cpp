#include <mbed.h>

#include <Arduino.h>
#include <Motors.h>

Motors::Motors()
: 
MotorADir(P0_4),
MotorBDir(P0_5),

MotorASpeed(P0_27),
MotorBSpeed(P1_2),

EncA(P1_11),
EncB(P1_12)
{
    ShaftRevA = 0;
    ShaftRevB = 0;

    EncCountA = 0;
    EncCountB = 0;
    desiredSpeed = 0.0f;
    currentSpeed = 0.0f;
    targetSpeed = 0.0f;

    step = 0.005f;
    pendingSpeedChange = false;
    pendingSpeed = 0.0f;
    currentLeftDir = 0;
    desiredLeftDir = 0;
    currentRightDir = 0;
    desiredRightDir = 0;
    isChangingDir = false;
    
};

// PUBLIC METHODS

void Motors::setup(mbed::InterruptIn &interrupt) {
    MotorASpeed.period_us(10);
    MotorBSpeed.period_us(10);

    MotorASpeed.write(0.0f);
    MotorBSpeed.write(0.0f);

    EncA.rise(mbed::callback(this, &Motors::countPulseA));
    EncB.rise(mbed::callback(this, &Motors::countPulseB));
    interrupt.fall(mbed::callback(this, &Motors::emergencyStop));
}  

void Motors::setTargetSpeed(float speed) {
    // Serial.println((String)"setTargetSpeed called while state = " + motorState);

    // if (motorState == CHANGING_DIR) {
    //   pendingSpeed = speed;
    //   pendingSpeedChange = true;
    //   return;
    // }

    // // Serial.println((String)"Speed set to: "+ speed);
    // desiredSpeed = speed;
    // targetSpeed = desiredSpeed;
    targetSpeed = speed;
    // if interrupt has caused state change
    if (motorState == CHANGING_DIR || motorState == EMERGENCY) return;

    if (speed > currentSpeed) {
      motorState = RAMP_UP;
    } else if (speed < currentSpeed) {
      motorState = RAMP_DOWN;
    } else if (speed == 0.0f) {
      motorState = STOPPED;
    }
    
  }

void Motors::setDirection(int leftdir, int rightdir) {
    // Motor A is Left Motor B is right
    desiredLeftDir = leftdir;
    desiredRightDir = rightdir;

    // set boolean if lef
    // bool directionChange = leftdir != currentLeftDir || rightdir != currentRightDir;

    // if (directionChange) {
    motorState = CHANGING_DIR;
    targetSpeed = 0.0f;
    // } 
}

void Motors::stop(){
    // leaves stateChange to targetspeed
    // Serial.println("SToppinger");
    targetSpeed = 0.0f;
    setTargetSpeed(0.0f);
}


void Motors::emergencyStop() {
    motorState = EMERGENCY;
    currentSpeed = 0.0f;
    targetSpeed = 0.0f;
    MotorBSpeed.write(currentSpeed);
    MotorASpeed.write(currentSpeed);
};



float Motors::getDistanceA() {
    float shaftRevs = ((float)EncCountA * 4.0) / (12 * 110);
    return shaftRevs * wheelCircumferance;
}


float Motors::getDistanceB() {
    float shaftRevs = ((float)EncCountB * 4.0) / (12 * 110);
    return shaftRevs * wheelCircumferance;
}

// PRIVATES
void Motors::countPulseA() {
    EncCountA++;
    if (EncCountA % (6 * 110) == 0) {

      ShaftRevA++;
    }
}

void Motors::countPulseB() {
    EncCountB++;
    if (EncCountB % (6 * 110) == 0) {

      ShaftRevB++;
    }
}

void Motors::handleRampUp() {
    // if (currentSpeed < targetSpeed) {
        // dont go above target
        currentSpeed += step;
    // } 

    // Serial.print((String)"CurrentSPeed = " + currentSpeed + " and current step = "+ step);
    if (currentSpeed >= targetSpeed) {
        currentSpeed = targetSpeed;
        motorState = RUNNING;
    }

    MotorASpeed.write(currentSpeed);
    MotorBSpeed.write(currentSpeed);
}

void Motors::handleRampDown() {
    // if (currentSpeed > targetSpeed) {
        currentSpeed -= step;
    // } 
    // Serial.println((String)"CurrentSPeed = " + currentSpeed + " and current step = "+ step);
    if (currentSpeed <= targetSpeed) {
        // Serial.println("First if");
        currentSpeed = targetSpeed;
        
        // Serial.println("Second if, setting state to STOPPED");
        motorState = currentSpeed == 0.0f ? STOPPED : RUNNING;
    }

    MotorASpeed.write(currentSpeed);
    MotorBSpeed.write(currentSpeed);
}

void Motors::handleChangingDir() {
    // speed down
    if (currentSpeed > 0.0f) {
        currentSpeed -= step;
        MotorASpeed.write(currentSpeed);
        MotorBSpeed.write(currentSpeed);
        return;
    }
    // apply new directions when stopped
    currentLeftDir = desiredLeftDir;
    currentRightDir = desiredRightDir;

    MotorADir = (currentLeftDir * -1) + 1;
    MotorBDir = currentRightDir;

    // check for pending speed then ramp up or stay idle
    if (pendingSpeedChange) {
        pendingSpeedChange = false;
        motorState = STOPPED; // set to stopped so targetSpeed runs
        setTargetSpeed(pendingSpeed); // ramp up again
    } else {
        motorState = STOPPED;
    }
}

void Motors::handleRunning() { 

    // check if state needs to change, else remain at current speed
    if (currentSpeed < targetSpeed) {
        // targetSpeed = desiredSpeed;
        motorState = RAMP_UP;
    }
    if (currentSpeed > targetSpeed) {
        // targetSpeed = desiredSpeed;
        motorState = RAMP_DOWN;
    }
    // reinforce
    
    currentSpeed = targetSpeed;
    setTargetSpeed(currentSpeed);
    // MotorASpeed.write(currentSpeed);
    // MotorBSpeed.write(currentSpeed);
    // Serial.println((String)"Current speed: " + currentSpeed);
}

void Motors::handleStopped() {
    
    currentSpeed = 0.0f;
    // reinforce speeds
    MotorASpeed.write(0.0f);
    MotorBSpeed.write(0.0f);
    if (desiredSpeed > 0.0f) {
        motorState = RAMP_UP;
    }
}


void Motors::handleEmergency() {
    currentSpeed = 0.0f;
    targetSpeed = 0.0f;
    MotorASpeed.write(currentSpeed);
    MotorBSpeed.write(currentSpeed);
}

const char* Motors::stateToString(STATES s) {
    switch (s) {
        case RUNNING: return "RUNNING";
        case RAMP_UP: return "RAMP_UP";
        case RAMP_DOWN: return "RAMP_DOWN";
        case STOPPED: return "STOPPED";
        case CHANGING_DIR: return "CHANGING_DIR";
        case EMERGENCY: return "EMERGENCY";
        default: return "UNKNOWN";
    };
}


void Motors::update() {
    if (stateToString(motorState) != "STOPPED") {
        if (printStatement) Serial.println((String)"Motor State: " + stateToString(motorState));
    }
    
    float diff = fabs(targetSpeed - currentSpeed);
    step = diff * 0.1f; // step is 20% of difference
    if (step < 0.01f) step = 0.01f;

    switch (motorState) {
        case RUNNING:
        handleRunning(); // done
        break;

        case RAMP_UP:
        handleRampUp(); // done
        break;
        
        case RAMP_DOWN:
        Serial.println((String)"Speed is "+ currentSpeed);
        handleRampDown(); // done
        break;

        case STOPPED:
        handleStopped(); // done
        break;

        case CHANGING_DIR:
        handleChangingDir(); 
        break;
        
        case EMERGENCY:
        handleEmergency();
        break;
    }

}