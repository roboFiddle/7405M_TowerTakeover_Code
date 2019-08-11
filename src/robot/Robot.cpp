//
// Created by alexweiss on 8/9/19.
//

#include "../lib/meecan_lib.hpp"
#include "../tests/testsInclude.hpp"
#include "main.h"
#include "Robot.hpp"

namespace meecan {

  Robot::Robot() {

  }

  void Robot::robotInit() {
    printf("robot init\n");
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Hello World!");
  }
  void Robot::disabledInit() {
    printf("disabled init\n");
  }
  void Robot::disabledLoop() {
    printf("disabled loop\n");
  }
  void Robot::autonomousInit() {
    printf("auton init\n");
  }
  void Robot::autonomousLoop() {
    printf("auton loop\n");
  }
  void Robot::driverInit() {
    printf("driver init\n");
    //test::testMeecanLib();
  }
  void Robot::driverLoop() {
    printf("driver loop\n");
  }
}