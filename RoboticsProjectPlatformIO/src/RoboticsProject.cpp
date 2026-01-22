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
Gyro gyro;
Sensors sensors(US, IR, gyro);

Controllers Controller(motors, gyro);

Exploration Explorer(sensors, Controller);


void setup() {
  // get clock speed error for timer
  sensors.setup();
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
    // Explorer.startExploring();

    Explorer.startExploring();
    // Controller.goToPose(200,200,0.0);
    // explorationActive = true;
  }
  if (JoystickDown == 0) {
    // Controllers::moveDistance(300.0, false);
    // Controller.requestTurnRightToCardinal();
    // Controller.goToPose(0, 1680, 180);
    Controller.requestTurn(180.0);
    
    // Controller.moveDistance(300, true);
  }
  if (JoystickLeft == 0) {
    // Controller.requestTurnLeftToCardinal();
    // Controller.goToPose(0, 0, 180);
    // Controller.getClosestCardinal(-46 * (PI/180));
    // Controller.align();
    // Controller.requestTurn(-90);
    Controller.moveDistance(200.0f, true);
  }
  // Serial.println("Blaaah"); 
  // Controller.update();
  // sensors.update();
  // if (print_data) {
  //   wait_us(500000);
  // }

}
