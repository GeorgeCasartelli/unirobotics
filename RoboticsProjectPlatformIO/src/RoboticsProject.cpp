#include <Arduino.h>
#include <mbed.h>
#include <Motors.h>
#include <Infrareds.h>
#include <Ultrasonics.h>
#include <Controllers.h>
#include <Exploration.h>
#include <Sensors.h>
#include <mbed.h>
// using namespace mbed;

mbed::DigitalIn JoystickUp(P0_28);
mbed::DigitalIn JoystickDown(P0_29);
mbed::DigitalIn JoystickLeft(P0_30);
mbed::InterruptIn JoystickRight(P0_3);

bool print_data = 0;
bool forwardFlag = 0;
static bool explorationActive = false;

Motors motors;
Infrareds IR;
Ultrasonics US;
Sensors sensors(US, IR);

Controllers Controller(motors);

Exploration Explorer(sensors, Controller);


void setup() {
  // get clock speed error for timer
  US.setup();
  motors.setup(JoystickRight);
  
}

void loop() {

  // sensors.update();

  // Serial.println((String)"Front: "+sensors.getFrontDist() + " RightIR: " + sensors.getRightDist_IR());

  Explorer.update();

  // else motors.stop();
  if (JoystickUp == 0/* && !explorationActive*/) {
    // Controller.moveDistance(300.0, true);
    // static int count = 0;
    // Serial.println((String)"StartExploring called " + (++count) + " times");
    Explorer.startExploring();
    // explorationActive = true;
  }
  if (JoystickDown == 0) {
    // Controllers::moveDistance(300.0, false);
    Controller.turnDegrees(90.0);
  }
  if (JoystickLeft == 0) {
    Controller.turnDegrees(-90.0);
  }
  // Serial.println("Blaaah"); 
  Controller.update();
  // if (print_data) {
  //   wait_us(500000);
  // }

}
