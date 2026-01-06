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
    stepMin = 0.0005f;
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
    if (motorState == EMERGENCY || motorState == CHANGING_DIR)
        return;
    targetSpeedLeft = left;
    targetSpeedRight = right;

}

void Motors::setDirection(int leftdir, int rightdir) {
    // Motor A is Left Motor B is right
    desiredLeftDir = leftdir;
    desiredRightDir = rightdir;
 
    transitionTo(CHANGING_DIR);
    targetSpeedLeft = 0.0f;
    targetSpeedRight = 0.0f;

}

void Motors::stop(){
    setTargetSpeeds(0.0f, 0.0f);
}


void Motors::emergencyStop() {
    transitionTo(EMERGENCY);
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

void Motors::transitionTo(STATES next) {
    if (motorState == next) return;
    motorState = next;
}



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

    transitionTo(STOPPED);
}

void Motors::handleStopped() {
    if (targetSpeedLeft > 0.0f || targetSpeedRight > 0.0f) { 
        transitionTo(RUNNING);
    }
}

void Motors::handleEmergency() {
    //nothing to see here!
}

const char* Motors::stateToString(STATES s) {
    switch (s) {
        case RUNNING: return "RUNNING";
        // case RAMP_UP: return "RAMP_UP";
        // case RAMP_DOWN: return "RAMP_DOWN";
        case STOPPED: return "STOPPED";
        case CHANGING_DIR: return "CHANGING_DIR";
        case EMERGENCY: return "EMERGENCY";
        default: return "UNKNOWN";
    };
}

void Motors::onEnterState(STATES state) {
    // actions on entry to states, to clean up main running handlers
    // one time actions
    switch (state) {
        case STOPPED:
            currentSpeedLeft = 0.0f;
            currentSpeedRight = 0.0f;
            MotorASpeed.write(currentSpeedLeft);
            MotorBSpeed.write(currentSpeedRight);
            break;

        case RUNNING:
            break;

        case CHANGING_DIR:
            targetSpeedLeft = 0.0f;
            targetSpeedRight = 0.0f;
            break;

        case EMERGENCY:
            currentSpeedLeft = 0.0f;
            currentSpeedRight = 0.0f;
            targetSpeedLeft = 0.0f;
            targetSpeedRight = 0.0f;
            MotorASpeed.write(0.0f);
            MotorBSpeed.write(0.0f);
            break;
    }
}

void Motors::update() {
    // do entry work if state has changed
    if (motorState != prevState) {
        onEnterState(motorState);
        prevState = motorState;
    }

    // debug prints
    if (motorState != STOPPED) {
        if (printStatement) Serial.println((String)"Motor State: " + stateToString(motorState));
    }
    
    
    // calculate speed errors
    float errorLeft = targetSpeedLeft - currentSpeedLeft;
    float errorRight = targetSpeedRight - currentSpeedRight;

    int leftFlag = errorLeft < 0 ? -1 : 1; // if less than zero, flag = -1
    int rightFlag = errorRight < 0 ? -1 : 1;
    // gives + or - step

    // if error is less than min step, step = error, else is 20% of error
    stepLeft = fabs(errorLeft) < stepMin ? fabs(errorLeft) : fabs(errorLeft) * 0.2f; 
    stepRight = fabs(errorRight) < stepMin ? fabs(errorRight) : fabs(errorRight) * 0.2f;


    // Serial.println(
    //     (String)"Values:\r\nerrorLeft: " + errorLeft + 
    //     "\r\nerrorRight: "+ errorRight+
    //     "\r\ncurrentSpeedLeft: "+currentSpeedLeft+
    //     "\r\ncurrentSpeedRight: "+currentSpeedRight+
    //     "\r\ntargetSpeedLeft: "+targetSpeedLeft+
    //     "\r\ntargetSpeedRight: "+targetSpeedRight+ 
    //     "\r\nstepLeft: "+stepLeft+
    //     "\r\nstepRight: "+stepRight
    // );


    switch (motorState) {
        case RUNNING: {
        // handleRunning(); // done
            // float deltaLeft = constrain(stepLeft, 0.05, 0.15);
            // float deltaRight = constrain(stepRight, 0.05, 0.15);

            currentSpeedLeft += stepLeft * leftFlag; // apply + or - flag to ramp up/down
            currentSpeedRight += stepRight * rightFlag;

            MotorASpeed.write(currentSpeedLeft);
            MotorBSpeed.write(currentSpeedRight);

            auto near = [](float a, float b, float eps) {
                return fabs(a - b) < eps;
            };
            bool atTarget = 
                near(currentSpeedLeft, targetSpeedLeft, stepMin) && 
                near(currentSpeedRight, targetSpeedRight, stepMin)
            ;

            bool targetisZero = (targetSpeedLeft == 0.0f) && (targetSpeedRight == 0.0f);

            if (atTarget && targetisZero) {
                transitionTo(STOPPED);
            }
            break;
        }

        case STOPPED: {

        handleStopped(); // done
        break;
        }

        case CHANGING_DIR: {

        handleChangingDir(); 
        break;
        }
        
        case EMERGENCY:{

        handleEmergency();
        break;
        }
    }

}