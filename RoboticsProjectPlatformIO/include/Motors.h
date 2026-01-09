#ifndef MOTOR_H
#define MOTOR_H

#include <mbed.h>
#include <Arduino.h>

class Motors{
    public:
        // methods
        Motors();
        void setup(mbed::InterruptIn &interrupt);
        void update();
        void setTargetSpeeds(float left, float right);
        void setDirection(int8_t left, int8_t right);
        void stop();
        void emergencyStop();
        float getDistanceA();
        float getDistanceB();


    private:
        // attributes
        mbed::DigitalOut MotorADir;
        mbed::DigitalOut MotorBDir;

        mbed::PwmOut MotorASpeed;
        mbed::PwmOut MotorBSpeed;

        mbed::InterruptIn EncA;
        mbed::InterruptIn EncB;

        uint32_t ShaftRevA = 0;
        uint32_t ShaftRevB = 0;

        volatile uint32_t EncCountA = 0;
        volatile uint32_t EncCountB = 0;

        const float wheelCircumferance = 3.14159f * 48;

        // float currentSpeed;
        // float targetSpeed;

        float currentSpeedLeft = 0.0f;
        float currentSpeedRight = 0.0f;
        float targetSpeedLeft = 0.0f;
        float targetSpeedRight = 0.0f;

        int8_t currentLeftDir = 0;
        int8_t currentRightDir = 0;
        int8_t desiredLeftDir = 0;
        int8_t desiredRightDir = 0;

        bool printStatement = false;

        //clock
        uint32_t lastUpdateUs = 0;

        // acceleration limits 
        float dt;
        float maxAccel = 1.2f; // 
        float maxDecel = 1.5f; // stronger breaking

        enum STATES {
            STOPPED,
            RUNNING,
            CHANGING_DIR,
            EMERGENCY
        };

        STATES motorState = STOPPED;
        STATES prevState = STOPPED;

        void transitionTo(STATES next);
        void onEnterState(STATES state);


        const char* stateToString(STATES s);

        // void ramp(float errorLeft, float errorRight);
        void ramp();
        void countPulseA();
        void countPulseB();
        void handleRampUp();
        void handleRampDown();
        void handleChangingDir();
        void handleRunning();
        void handleStopped();
        void handleEmergency();
        void constrainCurrentSpeeds();
};



#endif