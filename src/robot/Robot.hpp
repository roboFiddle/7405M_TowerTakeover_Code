//
// Created by alexweiss on 8/10/19.
//

#ifndef INC_7405M_CODE_SRC_ROBOT_ROBOT_HPP_
#define INC_7405M_CODE_SRC_ROBOT_ROBOT_HPP_

#include "../lib/meecan_lib.hpp"
#include "../tests/testsInclude.hpp"
#include "loops/Loop.hpp"
#include "loops/Looper.hpp"
#include "main.h"


namespace meecan {

  class Robot {
   private:
    loops::Looper* testLooper = 0;
   public:
    Robot();
    void robotInit();
    void disabledInit();
    void disabledLoop();
    void autonomousInit();
    void autonomousLoop();
    void driverInit();
    void driverLoop();

    /*struct RobotManager : public util::Singleton<Robot, RobotManager> {};
    static RobotManager instance;*/
    static Robot* instance;
  };
  Robot* Robot::instance = new Robot();
}

void initialize() {
  meecan::Robot::instance->robotInit();
}
void disabled() {
  meecan::Robot::instance->disabledInit();
  while(1) {
    meecan::Robot::instance->disabledLoop();
    pros::Task::delay(20);
  }
}
void autonomous() {
  meecan::Robot::instance->autonomousInit();
  while(1) {
    meecan::Robot::instance->autonomousLoop();
    pros::Task::delay(20);
  }
}
void opcontrol() {
  meecan::Robot::instance->driverInit();
  while(1) {
    meecan::Robot::instance->driverLoop();
    pros::Task::delay(20);
  }
}

#endif //INC_7405M_CODE_SRC_ROBOT_ROBOT_HPP_
