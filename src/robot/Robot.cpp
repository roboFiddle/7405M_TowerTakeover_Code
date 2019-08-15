//
// Created by alexweiss on 8/9/19.
//

#include "../lib/meecan_lib.hpp"
#include "../tests/testsInclude.hpp"
#include "main.h"
#include "Robot.hpp"
#include "Constants.hpp"
#include "auto/AutoModeRunner.hpp"
#include "auto/modes/TestMode.hpp"
#include "loops/Loop.hpp"
#include "loops/Looper.hpp"
#include "subsystems/Drive.hpp"

namespace meecan {
  Robot::RobotManager instance;

  Robot::Robot() {
    printf("robot construct\n");
    enabledLooper = new loops::Looper(0, "enabled");
    controller_ = new pros::Controller(pros::E_CONTROLLER_MASTER);
    setupMainLoop();
  }

  void Robot::robotInit() {
    mainLooper->enable();
    subsystems::Drive::instance->registerEnabledLoops(enabledLooper);
    std::shared_ptr<auton::AutoModeBase> activeMode(new auton::TestMode());
    auton::AutoModeRunner::instance->setAutoMode(activeMode);
  }
  void Robot::disabledInit() {
    enabledLooper->disable();
    auton::AutoModeRunner::instance->stop();
  }
  void Robot::disabledLoop() {
    //printf("%f, %f\n", subsystems::Drive::instance->getLeftVoltage(), subsystems::Drive::instance->getRightVoltage());
  }
  void Robot::autonomousInit() {
    enabledLooper->enable();
    auton::AutoModeRunner::instance->start();
  }
  void Robot::autonomousLoop() {
    //printf("%f, %f\n", subsystems::Drive::instance->getLeftVoltage(), subsystems::Drive::instance->getRightVoltage());
  }
  void Robot::driverInit() {
    auton::AutoModeRunner::instance->stop();
    enabledLooper->enable();
  }
  void Robot::driverLoop() {
    pros::lcd::print(0, "driver loop %d", pros::millis());
    units::Number throttle = controller_->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) / 127.0;
    units::Number turn = controller_->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) / 127.0;

    units::Number left = LIMIT(12*(throttle+turn), -12.0*units::num, 12.0*units::num);
    units::Number right = LIMIT(12*(throttle-turn), -12.0*units::num, 12.0*units::num);
    subsystems::Drive::instance->setOpenLoop(util::DriveSignal(left, right));
    //printf("%f, %f\n", subsystems::Drive::instance->getLeftVoltage(), subsystems::Drive::instance->getRightVoltage());
  }
}