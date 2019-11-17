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
#include "auto/modes/FrontAutoMode.hpp"
#include "auto/modes/FlipOutMode.hpp"
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
    std::shared_ptr<auton::AutoModeBase> activeMode(new auton::BackAutoMode(BACK_RED));
    auton::AutoModeRunner::instance->setAutoMode(activeMode);
    pros::lcd::initialize();
  }
  void Robot::disabledInit() {
    enabledLooper->disable();
    auton::AutoModeRunner::instance->stop();
  }
  void Robot::disabledLoop() {
    if(pros::lcd::read_buttons() == 4) {  //left
      while(pros::lcd::read_buttons() == 4) {pros::Task::delay(20);}
      current_auton--;
    }
    else if(pros::lcd::read_buttons() == 1) { //right
      while(pros::lcd::read_buttons() == 1) {pros::Task::delay(20);}
      current_auton++;
    }
    if(current_auton < 0)
      current_auton = 5;
    if(current_auton > 5)
      current_auton = 0;

    switch(current_auton) {
      case 0:
        pros::lcd::print(4, "DO NOTHING");
        break;
      case 1:
        pros::lcd::print(4, "RELEASE ONLY");
        break;
      case 2:
        pros::lcd::print(4, "SINGLE/BACK RED");
        break;
      case 3:
        pros::lcd::print(4, "SINGLE/BACK BLUE");
        break;
      case 4:
        pros::lcd::print(4, "DOUBLE/FRONT RED");
        break;
      case 5:
        pros::lcd::print(4, "DOUBLE/FRONT BLUE");
        break;
      default:
        pros::lcd::print(4, "YOU BROKE IT");
        break;
    }

  }
  void Robot::autonomousInit() {
    enabledLooper->enable();

    switch(current_auton) {
      case 1: {
        std::shared_ptr<auton::AutoModeBase> activeMode(new auton::FlipOutMode());
        auton::AutoModeRunner::instance->setAutoMode(activeMode); }
        break;
      case 2: {
        std::shared_ptr<auton::AutoModeBase> activeMode(new auton::BackAutoMode(BACK_RED));
        auton::AutoModeRunner::instance->setAutoMode(activeMode); }
        break;
      case 3: {
        std::shared_ptr<auton::AutoModeBase> activeMode(new auton::BackAutoMode(BACK_BLUE));
        auton::AutoModeRunner::instance->setAutoMode(activeMode); }
        break;
      case 4: {
        std::shared_ptr<auton::AutoModeBase> activeMode(new auton::FrontAutoMode(FRONT_RED));
        auton::AutoModeRunner::instance->setAutoMode(activeMode); }
        break;
      case 5: {
        std::shared_ptr<auton::AutoModeBase> activeMode(new auton::FrontAutoMode(FRONT_RED));
        auton::AutoModeRunner::instance->setAutoMode(activeMode); }
        break;
      default: {
        std::shared_ptr<auton::AutoModeBase> activeMode(new auton::DoNothingMode());
        auton::AutoModeRunner::instance->setAutoMode(activeMode); }
        break;
    }
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
    subsystems::Lift::instance->tare();
    subsystems::Tray::instance->setOpenLoop(0);
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


    turn = (turn.getValue() > 0 ? 1 : -1) * (turn / 200) * (turn / 200) * 200 ;
    if(subsystems::Drive::instance->getState() == subsystems::ControlState::OPEN_LOOP || std::fabs(throttle.getValue()) > 5 || std::fabs(turn.getValue()) > 5)
      subsystems::Drive::instance->setOpenLoop(util::DriveSignal(throttle+turn, throttle-turn));

    units::Number intake = 1.0*(controller_->get_digital(pros::E_CONTROLLER_DIGITAL_L2) - controller_->get_digital(pros::E_CONTROLLER_DIGITAL_L1));
    if(subsystems::Intake::instance->getState() == subsystems::ControlState::OPEN_LOOP || controller_->get_digital(pros::E_CONTROLLER_DIGITAL_L2) || controller_->get_digital(pros::E_CONTROLLER_DIGITAL_L1))
      subsystems::Intake::instance->setOpenLoop(intake * 200);

    units::Number tray = 1.0*(controller_->get_digital(pros::E_CONTROLLER_DIGITAL_A) - controller_->get_digital(pros::E_CONTROLLER_DIGITAL_Y));
    if(subsystems::Tray::instance->getState() == subsystems::ControlState::OPEN_LOOP || controller_->get_digital(pros::E_CONTROLLER_DIGITAL_A) || controller_->get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
      subsystems::Tray::instance->setOpenLoop(tray * constants::RobotConstants::MAX_TRAY_RPM);
    }
    printf("TRAY STATE %f, %d\n", tray.getValue(), subsystems::Tray::instance->getState() == subsystems::ControlState::OPEN_LOOP || controller_->get_digital(pros::E_CONTROLLER_DIGITAL_A) || controller_->get_digital(pros::E_CONTROLLER_DIGITAL_Y));


    if(controller_->get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
      std::shared_ptr<auton::AutoModeBase> activeMode(new auton::FlipOutMode());
      auton::AutoModeRunner::instance->setAutoMode(activeMode);
      auton::AutoModeRunner::instance->start();
      pros::Task::delay(4000);
    }

    units::Number lift = 1.0*(controller_->get_digital(pros::E_CONTROLLER_DIGITAL_R2) - controller_->get_digital(pros::E_CONTROLLER_DIGITAL_R1));
    if(subsystems::Lift::instance->getState() == subsystems::ControlState::OPEN_LOOP || controller_->get_digital(pros::E_CONTROLLER_DIGITAL_R1) || controller_->get_digital(pros::E_CONTROLLER_DIGITAL_R2) || controller_->get_digital(pros::E_CONTROLLER_DIGITAL_A) || controller_->get_digital(pros::E_CONTROLLER_DIGITAL_Y))
      subsystems::Lift::instance->setOpenLoop(lift * constants::RobotConstants::MAX_LIFT_RPM);

    /* if(controller_->get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))
      subsystems::Lift::instance->setPosition(constants::RobotConstants::LIFT_PRESETS[0]);
    if(controller_->get_digital(pros::E_CONTROLLER_DIGITAL_LEFT))
      subsystems::Lift::instance->setPosition(constants::RobotConstants::LIFT_PRESETS[1]);
    if(controller_->get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT))
      subsystems::Lift::instance->setPosition(constants::RobotConstants::LIFT_PRESETS[2]);
    if(controller_->get_digital(pros::E_CONTROLLER_DIGITAL_UP))
      subsystems::Lift::instance->setPosition(constants::RobotConstants::LIFT_PRESETS[3]); */

    if(controller_->get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))
      lift_state = 0;
    else if(controller_->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))
      lift_state--;
    else if(controller_->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
      lift_state ++;

    if(lift_state < 0) lift_state = 0;
    if(lift_state > 3) lift_state = 3;
    if(controller_->get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) || controller_->get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)  || controller_->get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) )
      subsystems::Lift::instance->setPosition(constants::RobotConstants::LIFT_PRESETS[lift_state]);

    if(controller_->get_digital(pros::E_CONTROLLER_DIGITAL_B))
      subsystems::Lift::instance->tare();
  }
}
