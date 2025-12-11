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
        void setTargetSpeed(float speed);
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

        long int EncCountA;
        long int EncCountB;

        const float wheelCircumferance = 3.14159f * 48;

        float desiredSpeed;
        float currentSpeed;
        float targetSpeed;
        float step;

        bool pendingSpeedChange;
        float pendingSpeed;

        int currentLeftDir;
        int currentRightDir;
        int desiredLeftDir;
        int desiredRightDir;
        bool isChangingDir = false;

        bool printStatement = false;

        //control
        float Kp = 0.01f; // proportional gain

        enum STATES {
            STOPPED,
            RAMP_UP,
            RAMP_DOWN,
            RUNNING,
            CHANGING_DIR,
            EMERGENCY
        };

        STATES motorState = STOPPED;

        const char* stateToString(STATES s);

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