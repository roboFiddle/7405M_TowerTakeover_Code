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

  Robot::Robot() {
    printf("robot construct\n");
    testLooper = new loops::Looper(0, "test");
    loops::Loop *test = new loops::Loop();

    test->onStart = [](){
      printf("test start\n");
    };
    test->onLoop = [](){
      printf("test loop\n");
    };
    test->onDisable = [](){
      printf("test disable\n");
    };

    std::shared_ptr<loops::Loop> ptr(test);
    testLooper->add(ptr);
  }

  void Robot::robotInit() {
    printf("robot init\n");
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Hello World!");
  }
  void Robot::disabledInit() {
    printf("disabled init \n");
    //testLooper->disable();
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
    testLooper->enable();
  }
  void Robot::driverLoop() {
    printf("driver loop\n");
  }
}