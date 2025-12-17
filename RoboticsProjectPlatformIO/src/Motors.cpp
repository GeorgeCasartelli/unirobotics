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
    
    currentSpeedLeft = 0.0f;
    targetSpeedLeft = 0.0f;

    currentSpeedRight = 0.0f;
    targetSpeedRight = 0.0f;

    stepLeft = 0.005f;
    stepRight = 0.005f;
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

void Motors::setTargetSpeeds(float left, float right) {
    targetSpeedLeft = left;
    targetSpeedRight = right;

    if (motorState == CHANGING_DIR || motorState == EMERGENCY) return;

    float avgTarget = (left + right) / 2.0f;
    float avgCurrent = (currentSpeedLeft + currentSpeedRight) / 2.0f;

    if (avgTarget > avgCurrent) {
        motorState = RAMP_UP;
    } else if (avgTarget < avgCurrent) {
        motorState = RAMP_DOWN;
    } else if (avgTarget == 0.0f) {
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
    targetSpeedLeft = 0.0f;
    targetSpeedRight = 0.0f;
    // } 
}

void Motors::stop(){
    // leaves stateChange to targetspeed
    // Serial.println("SToppinger");
    targetSpeedLeft = 0.0f;
    targetSpeedRight = 0.0f;
    setTargetSpeeds(0.0f, 0.0f);
}


void Motors::emergencyStop() {
    motorState = EMERGENCY;
    currentSpeedLeft = 0.0f;
    currentSpeedRight = 0.0f;
    targetSpeedLeft = 0.0f;
    targetSpeedRight = 0.0f;
    MotorBSpeed.write(currentSpeedRight);
    MotorASpeed.write(currentSpeedLeft);
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
    if (currentSpeedLeft < targetSpeedLeft) {
        currentSpeedLeft += stepLeft;
        if (currentSpeedLeft >= targetSpeedLeft) {
            currentSpeedLeft = targetSpeedLeft;
        }
    }

    if (currentSpeedRight < targetSpeedRight) {
        currentSpeedRight += stepRight;
        if (currentSpeedRight >= targetSpeedRight) {
            currentSpeedRight = targetSpeedRight;
        }
    }

    MotorASpeed.write(targetSpeedLeft);
    MotorBSpeed.write(targetSpeedRight);

    if (currentSpeedLeft >= targetSpeedLeft && currentSpeedRight >= targetSpeedRight) {
        motorState = RUNNING;
    }
}

void Motors::handleRampDown() {
    if (currentSpeedLeft > targetSpeedLeft) {
        currentSpeedLeft -= stepLeft;
        if (currentSpeedLeft <= targetSpeedLeft) {
            currentSpeedLeft = targetSpeedLeft;
        }
    }

    if (currentSpeedRight > targetSpeedRight) {
        currentSpeedRight -= stepRight;
        if (currentSpeedRight <= targetSpeedRight) {
            currentSpeedRight = targetSpeedRight;
        }
    }

    MotorASpeed.write(targetSpeedLeft);
    MotorBSpeed.write(targetSpeedRight);

    if (currentSpeedLeft <= targetSpeedLeft && currentSpeedRight <= targetSpeedRight) {
        bool bothStopped = (currentSpeedLeft == 0.0f && currentSpeedRight == 0.0f);
        motorState = bothStopped ? STOPPED : RUNNING;
    }
}

void Motors::handleChangingDir() {
    // speed down
    if (currentSpeedLeft > 0.0f || currentSpeedRight > 0.0f) {
        if (currentSpeedLeft > 0.0f) {
            currentSpeedLeft -= stepLeft;
            if (currentSpeedLeft < 0.0f) currentSpeedLeft = 0.0f;
        }
        if (currentSpeedRight > 0.0f) {
            currentSpeedRight -= stepRight;
            if (currentSpeedRight < 0.0f) currentSpeedRight = 0.0f;
        }
        MotorASpeed.write(currentSpeedLeft);
        MotorBSpeed.write(currentSpeedRight);
        return;
    }
    
    // apply new directions when stopped
    currentLeftDir = desiredLeftDir;
    currentRightDir = desiredRightDir;

    MotorADir = (currentLeftDir * -1) + 1;
    MotorBDir = currentRightDir;

    motorState = STOPPED;
}

void Motors::handleRunning() { 

    float avgCurrent = (currentSpeedLeft + currentSpeedRight) / 2.0f;
    float avgTarget = (targetSpeedLeft + targetSpeedRight) / 2.0f;
    // check if state needs to change, else remain at current speed

    if (avgCurrent < avgTarget) {
        // targetSpeed = desiredSpeed;
        motorState = RAMP_UP;
    }
    if (avgCurrent> avgTarget) {
        // targetSpeed = desiredSpeed;
        motorState = RAMP_DOWN;
    }
    // reinforce
    
    currentSpeedLeft = targetSpeedLeft;
    currentSpeedRight = targetSpeedRight;
    // setTargetSpeeds(currentSpeedLeft, currentSpeedRight);
    MotorASpeed.write(currentSpeedLeft);
    MotorBSpeed.write(currentSpeedRight);
    // Serial.println((String)"Current speed: " + currentSpeed);
}

void Motors::handleStopped() {
    
    currentSpeedLeft = 0.0f;
    currentSpeedRight = 0.0f;
    // reinforce speeds
    MotorASpeed.write(0.0f);
    MotorBSpeed.write(0.0f);

    if (targetSpeedLeft > 0.0f || targetSpeedRight > 0.0f) { 
        motorState = RAMP_UP;
    }
}


void Motors::handleEmergency() {
    currentSpeedLeft = 0.0f;
    targetSpeedRight = 0.0f;
    MotorASpeed.write(currentSpeedLeft);
    MotorBSpeed.write(currentSpeedRight);
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
    

    float diffLeft = fabs(targetSpeedLeft - currentSpeedRight);
    float diffRight = fabs(targetSpeedRight - currentSpeedRight);
    stepLeft = diffLeft * 0.2f; // step is 20% of difference
    stepRight = diffRight * 0.2;
    // if (step < 0.01f) step = 0.01f;
    // step = 0.2;

    switch (motorState) {
        case RUNNING:
        handleRunning(); // done
        break;

        case RAMP_UP:
        handleRampUp(); // done
        break;
        
        case RAMP_DOWN:
        // Serial.println((String)"Speed is "+ currentSpeed);
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