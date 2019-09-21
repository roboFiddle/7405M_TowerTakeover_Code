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
#include "paths/TrajectorySet.hpp"
#include "subsystems/Drive.hpp"
#include "subsystems/Odometry.hpp"

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
    subsystems::Odometry::instance->registerEnabledLoops(enabledLooper);
    std::shared_ptr<auton::AutoModeBase> activeMode(new auton::TestMode());
    auton::AutoModeRunner::instance->setAutoMode(activeMode);
    pros::lcd::initialize();
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
    geometry::Pose2d curPos = subsystems::Odometry::instance->getPosition();
    pros::lcd::print(1, "%f %f %f", curPos.translation().x().Convert(units::inch), curPos.translation().y().Convert(units::inch), curPos.rotation().getDegrees());
  }
  void Robot::driverInit() {
    auton::AutoModeRunner::instance->stop();
    enabledLooper->enable();


  }
  void Robot::driverLoop() {
    pros::lcd::print(0, "driver loop %d", pros::millis());
    geometry::Pose2d curPos = subsystems::Odometry::instance->getPosition();
    pros::lcd::print(1, "%f %f %f", curPos.translation().x().Convert(units::inch), curPos.translation().y().Convert(units::inch), curPos.rotation().getDegrees());
    units::Number throttle = controller_->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) / 127.0;
    units::Number turn = controller_->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) / 127.0;

    units::Number left = 200*(throttle+turn);
    units::Number right = 200*(throttle-turn);

    subsystems::Drive::instance->setOpenLoop(util::DriveSignal(left, right));
    //printf("%f, %f\n", subsystems::Drive::instance->getLeftVoltage(), subsystems::Drive::instance->getRightVoltage());
  }
}