//
// Created by alexweiss on 8/9/19.
//

#include "../lib/meecan_lib.hpp"
#include "../tests/testsInclude.hpp"
#include "main.h"
#include "Robot.hpp"
#include "Constants.hpp"
#include "loops/Loop.hpp"
#include "loops/Looper.hpp"

namespace meecan {
  Robot::RobotManager instance;

  Robot::Robot() {
    printf("robot construct\n");
    setupMainLoop();
  }

  void Robot::robotInit() {
    pros::lcd::initialize();
    pros::lcd::print(2, "robot init");
    mainLooper->enable();
  }
  void Robot::disabledInit() {
    pros::lcd::print(1, "disabled init %d", pros::millis());
  }
  void Robot::disabledLoop() {
    pros::lcd::print(0, "disabled loop %d", pros::millis());
  }
  void Robot::autonomousInit() {
    pros::lcd::print(1, "auton init %d", pros::millis());
  }
  void Robot::autonomousLoop() {
    pros::lcd::print(0, "auton loop %d", pros::millis());
  }
  void Robot::driverInit() {
    pros::lcd::print(1, "driver init %d", pros::millis());
  }
  void Robot::driverLoop() {
    pros::lcd::print(0, "driver loop %d", pros::millis());
  }
}