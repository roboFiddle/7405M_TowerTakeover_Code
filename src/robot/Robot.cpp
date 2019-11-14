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
#include "auto/modes/DoNothingMode.hpp"
#include "auto/modes/TestTrajectoryMode.hpp"
#include "auto/modes/BackAutoMode.hpp"
#include "loops/Loop.hpp"
#include "loops/Looper.hpp"
#include "paths/DriveMotionPlanner.hpp"
#include "paths/TrajectorySet.hpp"
#include "subsystems/Drive.hpp"
#include "subsystems/Odometry.hpp"
#include "subsystems/Intake.hpp"
#include "subsystems/Tray.hpp"
#include "subsystems/Lift.hpp"

namespace meecan {
  Robot::RobotManager instance;

  Robot::Robot() {
    printf("robot construct\n");
    enabledLooper = new loops::Looper(0, "enabled");
    controller_ = new pros::Controller(pros::E_CONTROLLER_MASTER);
    setupMainLoop();
  }

  void Robot::robotInit() {
    printf("robot init\n");
    mainLooper->enable();
    subsystems::Drive::instance->registerEnabledLoops(enabledLooper);
    subsystems::Odometry::instance->registerEnabledLoops(enabledLooper);
    subsystems::Intake::instance->registerEnabledLoops(enabledLooper);
    subsystems::Tray::instance->registerEnabledLoops(enabledLooper);
    subsystems::Lift::instance->registerEnabledLoops(enabledLooper);
    path_planning::TrajectorySet::instance->generatorCalls();
    std::shared_ptr<auton::AutoModeBase> activeMode(new auton::BackAutoMode());
    auton::AutoModeRunner::instance->setAutoMode(activeMode);
    pros::lcd::initialize();
  }
  void Robot::disabledInit() {
    enabledLooper->disable();
    auton::AutoModeRunner::instance->stop();
  }
  void Robot::disabledLoop() {
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
    units::Number throttle = controller_->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0 * 200.0;
    units::Number turn = controller_->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0 * 200.0;
    if(std::fabs(throttle.getValue()) < 20)
      throttle = 0;
    if(std::fabs(turn.getValue()) < 20)
        turn = 0;

    if(subsystems::Drive::instance->getState() == subsystems::ControlState::OPEN_LOOP || std::fabs(throttle.getValue()) > 5 || std::fabs(turn.getValue()) > 5)
    subsystems::Drive::instance->setOpenLoop(util::DriveSignal(throttle+turn, throttle-turn));

    units::Number intake = 1.0*(controller_->get_digital(pros::E_CONTROLLER_DIGITAL_L2) - controller_->get_digital(pros::E_CONTROLLER_DIGITAL_L1));
    if(subsystems::Intake::instance->getState() == subsystems::ControlState::OPEN_LOOP || controller_->get_digital(pros::E_CONTROLLER_DIGITAL_L2) || controller_->get_digital(pros::E_CONTROLLER_DIGITAL_L1))
      subsystems::Intake::instance->setOpenLoop(intake * 200);

    units::Number tray = 1.0*(controller_->get_digital(pros::E_CONTROLLER_DIGITAL_A) - controller_->get_digital(pros::E_CONTROLLER_DIGITAL_Y));
    if(subsystems::Tray::instance->getState() == subsystems::ControlState::OPEN_LOOP || controller_->get_digital(pros::E_CONTROLLER_DIGITAL_A) || controller_->get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
      subsystems::Tray::instance->setOpenLoop(tray * constants::RobotConstants::MAX_TRAY_RPM);
    }


    if(controller_->get_digital(pros::E_CONTROLLER_DIGITAL_X))
      subsystems::Tray::instance->activateScore(); // SCORE MACRO

    units::Number lift = 1.0*(controller_->get_digital(pros::E_CONTROLLER_DIGITAL_R2) - controller_->get_digital(pros::E_CONTROLLER_DIGITAL_R1));
    if(subsystems::Lift::instance->getState() == subsystems::ControlState::OPEN_LOOP || controller_->get_digital(pros::E_CONTROLLER_DIGITAL_R1) || controller_->get_digital(pros::E_CONTROLLER_DIGITAL_R2) || controller_->get_digital(pros::E_CONTROLLER_DIGITAL_A) || controller_->get_digital(pros::E_CONTROLLER_DIGITAL_Y))
      subsystems::Lift::instance->setOpenLoop(lift * constants::RobotConstants::MAX_LIFT_RPM);

    if(controller_->get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))
      subsystems::Lift::instance->setPosition(0);
    if(controller_->get_digital(pros::E_CONTROLLER_DIGITAL_LEFT))
      subsystems::Lift::instance->setPosition(constants::RobotConstants::LIFT_PRESETS[0]);
    if(controller_->get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT))
      subsystems::Lift::instance->setPosition(constants::RobotConstants::LIFT_PRESETS[1]);
    if(controller_->get_digital(pros::E_CONTROLLER_DIGITAL_UP))
      subsystems::Lift::instance->setPosition(constants::RobotConstants::LIFT_PRESETS[2]);
  }
}
