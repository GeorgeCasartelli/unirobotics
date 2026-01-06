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
        void setDirection(int left, int right);
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

        long int ShaftRevA;
        long int ShaftRevB;

        volatile long int EncCountA;
        volatile long int EncCountB;

        const float wheelCircumferance = 3.14159f * 48;

        // float currentSpeed;
        // float targetSpeed;

        float currentSpeedLeft;
        float currentSpeedRight;
        float targetSpeedLeft;
        float targetSpeedRight;

        float stepLeft;
        float stepRight;
        float stepMin;

        bool pendingSpeedChange;
        float pendingSpeed;

        int currentLeftDir;
        int currentRightDir;
        int desiredLeftDir;
        int desiredRightDir;
        bool isChangingDir = false;

        bool printStatement = true;

        //control
        float Kp = 0.01f; // proportional gain

        enum STATES {
            STOPPED,
            // RAMP_UP,
            // RAMP_DOWN,
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
};



#endif