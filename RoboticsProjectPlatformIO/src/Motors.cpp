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
{ };

// PUBLIC METHODS

void Motors::setup(mbed::InterruptIn &interrupt) {
    MotorASpeed.period_us(10);
    MotorBSpeed.period_us(10);

    MotorASpeed.write(0.0f);
    MotorBSpeed.write(0.0f);

    EncA.rise(mbed::callback(this, &Motors::countPulseA));
    EncB.rise(mbed::callback(this, &Motors::countPulseB));
    interrupt.fall(mbed::callback(this, &Motors::emergencyStop));

    lastUpdateUs = micros();
}  

void Motors::setTargetSpeeds(float left, float right) {
    if (motorState == EMERGENCY || motorState == CHANGING_DIR)
        return;
    targetSpeedLeft = left;
    targetSpeedRight = right;

}

void Motors::setDirection(int8_t leftdir, int8_t rightdir) {
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


void Motors::handleRunning() {
    // calculate speed errors
    float errorLeft = targetSpeedLeft - currentSpeedLeft;
    float errorRight = targetSpeedRight - currentSpeedRight;

    float accelL = (errorLeft > 0) ? maxAccel : maxDecel;
    float accelR = (errorRight > 0) ? maxAccel : maxDecel;

    float deltaL = accelL * dt;
    float deltaR = accelR * dt;

    float changeL = constrain(errorLeft, -deltaL, deltaL);
    float changeR = constrain(errorRight, -deltaR, deltaR);

    currentSpeedLeft += changeL;
    currentSpeedRight += changeR;

    constrainCurrentSpeeds();

    MotorASpeed.write(currentSpeedLeft);
    MotorBSpeed.write(currentSpeedRight);

    bool atTarget = 
        fabs(errorLeft)        < 0.00001f &&
        fabs(errorRight)       < 0.00001f
    ;

    bool targetisZero = 
        fabs(targetSpeedLeft)  < 0.00001f && 
        fabs(targetSpeedRight) < 0.00001f
    ;

    if (atTarget && targetisZero) {
        transitionTo(STOPPED);
    }
}

void Motors::handleChangingDir() {
    // speed down
    float decelL = maxDecel * dt;
    float decelR = maxDecel * dt;

    bool moving = false;
    
    if (currentSpeedLeft > 0.0f) {
        currentSpeedLeft -= decelL;
        if (currentSpeedLeft < 0.0f) currentSpeedLeft = 0.0f;
        moving = true;
    }

    if (currentSpeedRight > 0.0f) {
        currentSpeedRight -= decelR;
        if (currentSpeedRight < 0.0f) currentSpeedRight = 0.0f;
        moving = true;
    }

    constrainCurrentSpeeds();

    MotorASpeed.write(currentSpeedLeft);
    MotorBSpeed.write(currentSpeedRight);
    
    if (moving) return;
    

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

void Motors::constrainCurrentSpeeds(){
    // function to clamp speeds between 0 and 1
    currentSpeedLeft = constrain(currentSpeedLeft, 0.0f, 1.0f);
    currentSpeedRight = constrain(currentSpeedRight, 0.0f, 1.0f);
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


    //calculate dt
    uint32_t now = micros();
    dt = (now - lastUpdateUs) * 1e-6; //calculate dt in us
    lastUpdateUs = now;

    if (dt <= 0.0f || dt > 0.1f) { // keep above zero and ignore pauses
        dt = 0.0f;
    }
    // do entry work if state has changed
    if (motorState != prevState) {
        onEnterState(motorState);
        prevState = motorState;
    }

    // debug prints
    if (motorState != STOPPED) {
        if (printStatement) Serial.println((String)"Motor State: " + stateToString(motorState));
    }


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
            handleRunning(); // done
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