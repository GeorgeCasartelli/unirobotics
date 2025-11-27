#include <mbed.h>
using namespace mbed;

const char MUX_CMD[4] = { 0x01, 0x02, 0x04, 0x08 };

const char mux_addr = 0xEE;

float us_distances[2];


DigitalIn JoystickUp(P0_28);
DigitalIn JoystickDown(P0_29);
DigitalIn JoystickLeft(P0_30);
InterruptIn JoystickRight(P0_3);

I2C i2c(P0_31, P0_2);

bool print_data = 0;
bool forwardFlag = 0;


namespace IR {
  const char I2C_ADDR = 0x80;
  const char DISTANCE_ADDR = 0x5E;
  char data[2];

  float distances[4];

  void runtime() {
    // for all sensors
    for (int i = 0; i <= 3; i++) {

      i2c.write(mux_addr, &MUX_CMD[i], 1);
      i2c.write(I2C_ADDR, &DISTANCE_ADDR, 1);  // write 1st byte of cmd to addr 0x80
      wait_us(70);                             // wait x us

      data[0] = 0;
      data[1] = 0;

      int result = i2c.read(I2C_ADDR, data, 2);  // read 2 bytes

      // since i2c returns 0 on success (ack)
      if (result != 0) {
        distances[i] = 0.0f;
        continue;
      }

      // create 12 bit distance variable (bit shifted 8 bit + 4 bit)
      float distance = ((((data[0] << 4) | (data[1])) >> 1) * 0x35) / 16.0f;

      distances[i] = distance;
    }
  }
}


namespace US {
  mbed::DigitalInOut US_1(P0_23);
  mbed::DigitalInOut US_2(P1_14);
  mbed::DigitalInOut US_3(P1_13);
  mbed::DigitalInOut US_4(P1_15);

  mbed::DigitalInOut* sensors[4] = {
    // array of pointers for pins to not copy vals
    &US_1,
    &US_2,
    &US_3,
    &US_4
  };

  mbed::Timer timer;

  int correction = 0;
  float distances[4];

  void runtime(int val) {

    sensors[val]->output();
    *sensors[val] = 0;  // ensure pin is low
    *sensors[val] = 1;
    timer.reset();
    wait_us(10);        // wait 10us
    *sensors[val] = 0;  // set low

    sensors[val]->input();
    while (*sensors[val] == 0) {};
    timer.start();
    while (*sensors[val] == 1) {};
    timer.stop();

    distances[val] = (timer.read_us() - correction) / 58.0;  // formula from datasheet

    if (print_data) {
      Serial.println((String) "Ultrasonic Distance " + val + ": " + distances[val]);
    }
  }
}


namespace Motors {
  mbed::DigitalOut MotorADir(P0_4);
  mbed::DigitalOut MotorBDir(P0_5);

  mbed::PwmOut MotorASpeed(P0_27);
  mbed::PwmOut MotorBSpeed(P1_2);

  mbed::InterruptIn EncA(P1_11);
  long int ShaftRevA;
  long int EncCountA;

  mbed::InterruptIn EncB(P1_12);
  long int ShaftRevB;
  long int EncCountB;

  float wheelCircumference = 3.14159f * 48;

  float desiredSpeed = 0.0f;
  float currentSpeed = 0.0f;
  float targetSpeed = 0.0f;
  float step = 0.02f;

  bool pendingSpeedChange = false;
  float pendingSpeed = 0.0f;

  const float stoppingThreshold = 0.005;

  int currentLeftDir = 0;
  int desiredLeftDir = 0;
  int currentRightDir = 0;
  int desiredRightDir = 0;
  bool isChangingDir = false;

  void countPulseA();
  void countPulseB();
  void emergencyStop();

  enum STATES {
    STOPPED,
    RAMP_UP,
    RAMP_DOWN,
    RUNNING,
    CHANGING_DIR,
    EMERGENCY
  };

  STATES motorState = STOPPED;

  void setup() {
    MotorASpeed.period_us(10);
    MotorBSpeed.period_us(10);

    MotorASpeed.write(0.0f);
    MotorBSpeed.write(0.0f);

    EncA.rise(&countPulseA);
    EncB.rise(&countPulseB);
    JoystickRight.fall(&emergencyStop);
  }

  void setDirection(int leftdir, int rightdir) {
    // Motor A is Left Motor B is right
    desiredLeftDir = leftdir;
    desiredRightDir = rightdir;

    // set boolean if lef
    bool directionChange = leftdir != currentLeftDir || rightdir != currentRightDir;

    // if (directionChange) {
    motorState = CHANGING_DIR;
    targetSpeed = 0.0f;
    // } 
  }

  void setTargetSpeed(float speed) {
    Serial.println((String)"setTargetSpeed called while state = " + motorState);

    if (motorState == CHANGING_DIR) {
      pendingSpeed = speed;
      pendingSpeedChange = true;
      return;
    }

    // if (motorState == STOPPED) {
    //   Serial.println("MotorState == STOPPED");
    //   return;
    // }

    Serial.println((String)"Speed set to: "+ speed);
    desiredSpeed = speed;
    // if (motorState == STOPPED || motorState == RUNNING) {

    targetSpeed = desiredSpeed;
    if (speed > currentSpeed) {
      motorState = RAMP_UP;
    } else if (speed < currentSpeed) {
      motorState = RAMP_DOWN;
    } else if (speed == 0.0f) {
      motorState = STOPPED;
    }
    // }
  }

  void stop() {
    // Serial.println("Slow stop");
    // motorState = RAMP_DOWN;

    desiredSpeed = 0.0f;
    setTargetSpeed(0.0f);
  }

  void emergencyStop() {
    motorState = EMERGENCY;
    currentSpeed = 0.0f;
    targetSpeed = 0.0f;
    MotorBSpeed.write(currentSpeed);
    MotorASpeed.write(currentSpeed);
  }

  void countPulseA() {
    EncCountA++;
    if (EncCountA % (6 * 110) == 0) {

      ShaftRevA++;
    }
  }

  void countPulseB() {
    EncCountB++;
    if (EncCountB % (6 * 110) == 0) {

      ShaftRevB++;
    }
  }

  float getDistanceA() {
    // 660 is edges per shaft * gear ratio
    float shaftRevs = ((float)EncCountA * 4.0) / (12 * 110);
    return shaftRevs * wheelCircumference;
  }

  float getDistanceB() {
    float shaftRevs = ((float)EncCountB * 4.0) / (12 * 110);
    return shaftRevs * wheelCircumference;
  }

  void handleRampUp() {
    if (currentSpeed < targetSpeed) {
      // dont go above target
      currentSpeed += step;
    } 

    if (currentSpeed >= targetSpeed) {
      currentSpeed = targetSpeed;
      motorState = RUNNING;
    }

    MotorASpeed.write(currentSpeed);
    MotorBSpeed.write(currentSpeed);
  }

  void handleRampDown() {
    if (currentSpeed > targetSpeed) {
      currentSpeed -= step;
    } 

    if (currentSpeed <= targetSpeed) {
      Serial.println("First if");
      currentSpeed = targetSpeed;
      if (currentSpeed == 0.0f) {
        Serial.println("Second if, setting state to STOPPED");
        motorState = STOPPED;
      }
      // else {
      //   Serial.println("Setting bakc to running");
      //   motorState = RUNNING;
      // }
    }

    MotorASpeed.write(currentSpeed);
    MotorBSpeed.write(currentSpeed);
  }

  void handleChangingDir() {
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
    // MotorADir = currentLeftDir;
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

  void handleRunning() { 

    if (currentSpeed < desiredSpeed) {
      targetSpeed = desiredSpeed;
      motorState = RAMP_UP;
    }
    if (currentSpeed > desiredSpeed) {
      targetSpeed = desiredSpeed;
      motorState = RAMP_DOWN;
    }
    // reinforce speed
    // MotorASpeed.write(currentSpeed);
    // MotorBSpeed.write(currentSpeed);
  }

  void handleStopped() {
    if (desiredSpeed > 0.0f) {
      motorState = RAMP_UP;
    }
    currentSpeed = 0.0f;
    // reinforce speeds
    MotorASpeed.write(0.0f);
    MotorBSpeed.write(0.0f);
  }


  void handleEmergency() {
    currentSpeed = 0.0f;
    targetSpeed = 0.0f;
    MotorASpeed.write(currentSpeed);
    MotorBSpeed.write(currentSpeed);
  }

  void update() {
    
    switch (motorState) {
      case RUNNING:
        Serial.println("RUNNING");
        handleRunning(); // done
        break;

      case RAMP_UP:
        Serial.println("RAMP_UP");
        handleRampUp(); // done
        break;
      
      case RAMP_DOWN:
        Serial.println("RAMP_DOWN");
        handleRampDown(); // done
        break;

      case STOPPED:
        Serial.println("STOPPED");
        handleStopped(); // done
        break;

      case CHANGING_DIR:
        Serial.println("CHANGING_DIR");
        handleChangingDir(); 
        break;
      
      case EMERGENCY:
        Serial.println("EMERGENCY");
        handleEmergency();
        break;
    }

  }
}

namespace Controllers {
  float distanceTarget = 0.0f;
  float startDistanceA = 0.0f;
  float startDistanceB = 0.0f;
  bool movingForward = true;


  void reset();

  void moveDistance(float target, bool forward) {
    // set start vals to distance of motors
    startDistanceA = Motors::getDistanceA();
    startDistanceB = Motors::getDistanceB();
    float avg_distance = ((Motors::getDistanceA() - startDistanceA) + (Motors::getDistanceB() - startDistanceB)) / 2.0f;
    distanceTarget = target + avg_distance;

    movingForward = forward;

    
    if (movingForward) {  
      Motors::setDirection(1, 1);
    }
    else { 
      Motors::setDirection(0, 0);
    }
    // Serial.println("Move distance called and target speed set");
    Motors::setTargetSpeed(0.6f);
  }

  void turnDegrees(float angle) {
    
  }

  void update(){
    Motors::update();

    float avg_traveled = ((Motors::getDistanceA() - startDistanceA) + (Motors::getDistanceB() - startDistanceB)) / 2.0f;
    Serial.println((String)"Avg traveled: ?" + avg_traveled + "Distance Target: "+ distanceTarget);
    if (avg_traveled >= distanceTarget) {
      Serial.println("IF stament hit STOPPING MOTORS !");
      Motors::stop();
    }
  }

  void reset() {

  }
}


void setup() {
  // get clock speed error for timer
  US::timer.reset();
  US::timer.start();
  US::timer.stop();
  US::correction = US::timer.read_us();

  Motors::setup();
}


void loop() {

  // static unsigned long lastLog = 0;
  // unsigned long now = millis();

  // // log at ~50 Hz
  // if (now - lastLog >= 20) {
  //   lastLog = now;

  //   float a = Motors::getDistanceA();    // encoder A distance in mm
  //   float b = Motors::getDistanceB();    // encoder B distance in mm
  //   float avg = (a + b) / 2.0f;
  //   float ir_mm_a = IR::distances[0] / 10;         // front IR sensor in mm
  //   float ir_mm_b = IR::distances[1] / 10;
  //   float ir_avg = (ir_mm_a + ir_mm_b) / 2.0f;
  //   float us = US::distances[1] * 10;

  //   Serial.print(now); Serial.print(",");
  //   Serial.print(a);   Serial.print(",");
  //   Serial.print(b);   Serial.print(",");
  //   Serial.print(avg); Serial.print(",");
  //   Serial.print(ir_mm_a); Serial.print(",");
  //   Serial.print(ir_mm_b); Serial.print(",");
  //   Serial.print(ir_avg); Serial.print(",");
  //   Serial.println(us);
    

  // }
  // Serial.println("\r\n");

  IR::runtime();

  if (print_data) {
    Serial.println((String) "Distance 1: " + IR::distances[0] + "\r\nDistance 2: " + IR::distances[1] + "\r\nDistance 3: " + IR::distances[2] + "\r\nDistance 4: " + IR::distances[3]);
  }

  US::runtime(0);
  US::runtime(1);



  // float front = IR::distances[0];
  float left = US::distances[1];
  float right = US::distances[0];
  float front = IR::distances[0];


  // if (left < 10.0) {
  //   // Serial.println("Close left");
  //   Motors::setDirection(1, 0);
  //   Motors::setTargetSpeed(0.5f);
  // } else if (right < 10.0) {
  //   // Serial.println("Close right");
  //   Motors::setDirection(0, 1);
  //   Motors::setTargetSpeed(0.5f);
  // } else if (front < 600.0) {
  //   // Serial.println("Stopping");
  //   Motors::stop();
  // } else if (front < 1500.0) {
  //   // Serial.println("Moving forward");
  //   Motors::setDirection(1,1);
  //   Motors::setTargetSpeed(1.0f);
  // }

  // else {
  //   // Serial.println("Resetting");
  //   Motors::setDirection(1,1);
  //   Motors::setTargetSpeed(0.0f);
  // }


  // Motors::runtime();

  // if (JoystickUp == 0) {
  //   Motors::setDirection(1, 1);
  //   Motors::setTargetSpeed(1.0f);

  //   // Serial.println("Motors Forward");
  // } else if (JoystickDown == 0) {
  //   Motors::setDirection(0, 0);
  //   Motors::setTargetSpeed(1.0f);

  //   // Serial.println("Motors Backway");
  // } else if (JoystickLeft == 0) {
  //   Motors::setDirection(0, 1);
  //   Motors::setTargetSpeed(0.5f);
  //   Serial.println("Left");
  // } else if (JoystickRight == 0) {
  //   Motors::setDirection(1, 0);
  //   Motors::setTargetSpeed(0.5f);
  //   Serial.println("Right");
  // }

  // else Motors::stop();
  if (JoystickUp == 0) {
    Controllers::moveDistance(300.0, true);
  }
  if (JoystickDown == 0) {
    Controllers::moveDistance(300.0, false);
  }
  // if (Motors::getDistanceA() && Motors::getDistanceB() > 300.0) {
  //   Motors::stop();
  // } else {
  //   Motors::setTargetSpeed(1.0f);
  //   Motors::setDirection(1, 1);
  // }
  Controllers::update();
  if (print_data) {
    wait_us(500000);
  }

  // Serial.println((String) "Motor A: " + Motors::getDistanceA());
  // Serial.println((String) "Motor B: " + Motors::getDistanceB());

  // wait_us();
}
