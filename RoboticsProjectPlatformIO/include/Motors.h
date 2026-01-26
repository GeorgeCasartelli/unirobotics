#ifndef MOTOR_H
#define MOTOR_H

#include <mbed.h>
#include <Arduino.h>

class Motors{
    public:
        
        Motors();

        void setup(mbed::InterruptIn &interrupt);
        void update();

        // motion
        void setTargetSpeeds(float left, float right);
        void setDirection(int8_t left, int8_t right);
        void stop();
        void emergencyStop();

        // state queries
        bool isChangingDir() { return motorState == CHANGING_DIR; }
        bool isStopped() { return motorState == STOPPED; }

        // direction 
        int8_t getCurrentLeftDir() { return currentLeftDir; };
        int8_t getCurrentRightDir() { return currentRightDir; }

        // odom
        float getDistanceA();
        float getDistanceB();

    private:
        // pins
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

        // constants
        static constexpr float SPEED_TOLERANCE = 0.00001f;
        static constexpr float WHEEL_CIRCUMFERENCE = 3.14159f * 48;
        static constexpr uint16_t GEAR_RATIO = 110;
        static constexpr uint16_t ENCODER_PULSES = 12;
        static constexpr uint16_t PWM_PERIOD_US = 10;

        // acceleration limits 
        static constexpr float maxAccel = 1.2f; // 
        static constexpr float maxDecel = 1.5f; // stronger breaking

        // speed & direction ctrl
        float currentSpeedLeft = 0.0f;
        float currentSpeedRight = 0.0f;
        float targetSpeedLeft = 0.0f;
        float targetSpeedRight = 0.0f;
        int8_t currentLeftDir = 0;
        int8_t currentRightDir = 0;
        int8_t desiredLeftDir = 0;
        int8_t desiredRightDir = 0;


        //clock
        uint32_t lastUpdateUs = 0;
        float dt;

        // const float minPWM = 0.23f;
        // const float maxPWM = 1.0f;
        bool printStatement = false;

        enum STATES {
            STOPPED,
            RUNNING,
            CHANGING_DIR,
            EMERGENCY
        };

        STATES motorState = STOPPED;
        STATES prevState = STOPPED;

        void setState(STATES next);
        void onEnterState(STATES state);
        const char* stateToString(STATES s);

        // encoder callbacks
        void countPulseA();
        void countPulseB();

        // state handlers
        void handleChangingDir();
        void handleRunning();
        void handleStopped();
        void handleEmergency();
    
        void constrainCurrentSpeeds();
};



#endif