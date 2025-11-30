#include <Arduino.h>
#include <mbed.h>
#include <Motors.h>
#include <Infrareds.h>
#include <Ultrasonics.h>
#include <Controllers.h>
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

Controllers Controller(motors);


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
    Controller.moveDistance(300.0, true);
  }
  if (JoystickDown == 0) {
    // Controllers::moveDistance(300.0, false);
    Controller.turnDegrees(90.0);
  }
  if (JoystickLeft == 0) {
    Controller.turnDegrees(-90.0);
  }

  Controller.update();
  if (print_data) {
    wait_us(500000);
  }

}
