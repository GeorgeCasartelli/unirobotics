#include <Arduino.h>
#include <mbed.h>
#include <Motors.h>
#include <Infrareds.h>
#include <Ultrasonics.h>
#include <Controller.h>
#include <Exploration.h>
#include <Sensors.h>


mbed::DigitalIn JoystickUp(P0_28);
mbed::DigitalIn JoystickDown(P0_29);
mbed::DigitalIn JoystickLeft(P0_30);
mbed::InterruptIn JoystickRight(P0_3);

Motors motors;
Infrareds infrareds;
Ultrasonics ultrasonics;
Gyro gyro; // not used

Sensors sensors(ultrasonics, infrareds, gyro);
Controller controller(motors, gyro);
Exploration Explorer(sensors, controller);


void setup() {
  sensors.setup();
  motors.setup(JoystickRight);
}

void handleJoystick() {
  if (JoystickUp == 0) {
    // start maze!!!
    Explorer.startExploring();
  }
  if (JoystickDown == 0) {

    controller.requestTurn(180.0);

  }
  if (JoystickLeft == 0) {

    controller.moveDistance(200.0f, true);
  }
}

void loop() {
  Explorer.update();
  handleJoystick();
}
