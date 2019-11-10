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
    std::shared_ptr<auton::AutoModeBase> activeMode(new auton::TestTrajectoryMode());
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
    //auton::AutoModeRunner::instance->start();
    test::testTrajectory::newTest();

  }
  void Robot::autonomousLoop() {
    //printf("%f, %f\n", subsystems::Drive::instance->getLeftVoltage(), subsystems::Drive::instance->getRightVoltage());
    geometry::Pose2d curPos = subsystems::Odometry::instance->getPosition();
    pros::lcd::print(1, "%f %f %f", curPos.translation().x().Convert(units::inch), curPos.translation().y().Convert(units::inch), curPos.rotation().getDegrees());
  }
  void Robot::driverInit() {
    auton::AutoModeRunner::instance->stop();
    enabledLooper->enable();

    path_planning::TrajectorySet::instance->generatorCalls();
    auto timed_path = path_planning::TrajectorySet::instance->get("testSCurve").get(0);
    for(int i = 0; i < timed_path.length(); i++) {
      trajectory::TimedState<geometry::Pose2dWithCurvature> state = timed_path.getState(i);
      physics::DifferentialDrive::DriveDynamics
          dynamics = path_planning::DriveMotionPlanner::drive_model.solveInverseDynamics(
          physics::DifferentialDrive::ChassisState<units::QSpeed, units::QAngularSpeed>(state.velocity(),
                                                                                        state.velocity()
                                                                                            * state.state().curvature()),
          physics::DifferentialDrive::ChassisState<units::QAcceleration,
                                                   units::QAngularAcceleration>((state.acceleration()),
                                                                                state.acceleration()
                                                                                    * state.state().curvature())
      );
      printf("X (%f, %f, %f, %f, %f)\n", state.t(), dynamics.chassis_velocity.linear_, dynamics.chassis_velocity.angular_, dynamics.wheel_velocity.get(0), dynamics.wheel_velocity.get(1));
    }

  }
  void Robot::driverLoop() {
    pros::lcd::print(0, "driver loop %d", pros::millis());
    geometry::Pose2d curPos = subsystems::Odometry::instance->getPosition();
    pros::lcd::print(1, "%f %f %f", curPos.translation().x().Convert(units::inch), curPos.translation().y().Convert(units::inch), curPos.rotation().getDegrees());
    units::Number throttle = controller_->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0 * 200.0;
    units::Number turn = controller_->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0 * 200.0;

    subsystems::Drive::instance->setOpenLoop(util::DriveSignal(throttle+turn, throttle-turn));

    units::Number intake = 1.0*(controller_->get_digital(pros::E_CONTROLLER_DIGITAL_L1) - controller_->get_digital(pros::E_CONTROLLER_DIGITAL_L2));
    subsystems::Intake::instance->setOpenLoop(intake * 200);

    units::Number tray = 1.0*(controller_->get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) - controller_->get_digital(pros::E_CONTROLLER_DIGITAL_LEFT));
    subsystems::Tray::instance->setOpenLoop(tray * constants::RobotConstants::MAX_TRAY_RPM);

    units::Number lift = 1.0*(controller_->get_digital(pros::E_CONTROLLER_DIGITAL_R1) - controller_->get_digital(pros::E_CONTROLLER_DIGITAL_R2));
    subsystems::Lift::instance->setOpenLoop(lift * 200);
    //printf("%f, %f\n", subsystems::Drive::instance->getLeftVoltage(), subsystems::Drive::instance->getRightVoltage());
  }
}
