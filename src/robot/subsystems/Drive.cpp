//
// Created by alexweiss on 8/14/19.
//

#include "Drive.hpp"
#include "Odometry.hpp"
#include "../Constants.hpp"
#include <stdio.h>

namespace subsystems {
  Drive::DriveManager Drive::instance;

  Drive::Drive() {
    frontLeft = new pros::Motor(constants::RobotConstants::motor_drive_frontleft, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    frontRight = new pros::Motor(constants::RobotConstants::motor_drive_frontright, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
    backLeft = new pros::Motor(constants::RobotConstants::motor_drive_backleft, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    backRight = new pros::Motor(constants::RobotConstants::motor_drive_backright, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
    setBrakeMode(false);
    setOpenLoop(util::DriveSignal::NEUTRAL);
  }

  void Drive::stop()  {
    setOpenLoop(util::DriveSignal::NEUTRAL);
  }
  void Drive::zeroSensors() {
    frontLeft->tare_position();
    frontRight->tare_position();
    backLeft->tare_position();
    backRight->tare_position();

  }
  void Drive::setOpenLoop(util::DriveSignal signal) {
    if (currentState != ControlState::OPEN_LOOP) {
      currentState = ControlState::OPEN_LOOP;
      setBrakeMode(true);
    }

    left_demand = signal.left_voltage();
    right_demand = signal.right_voltage();
  }
  void Drive::setVelocity(util::DriveSignal velocity, util::DriveSignal feedforward) {
    if (currentState != ControlState::PATH_FOLLOWING) {
      currentState = ControlState::PATH_FOLLOWING;
      setBrakeMode(true);
    }

    left_demand = velocity.left_voltage();
    right_demand = velocity.right_voltage();
    left_feed_forward = feedforward.left_voltage();
    right_feed_forward = feedforward.right_voltage();
  }
  void Drive::registerEnabledLoops(loops::Looper* enabledLooper){
    loops::Loop* thisLoop = new loops::Loop();
    thisLoop->onStart = []() {};
    thisLoop->onLoop = []() {
      switch (Drive::instance->getState()) {
        case ControlState::OPEN_LOOP:
          break;
        case ControlState::PATH_FOLLOWING:
          Drive::instance->updatePathFollower();
          break;
        default:printf("Unexpected drive control state\n");
          break;
      }
      Drive::instance->updateOutputs();
    };
    thisLoop->onDisable = []() {};
    enabledLooper->add(std::shared_ptr<loops::Loop>(thisLoop));
  }
  void Drive::updatePathFollower() {
    if (currentState != ControlState::PATH_FOLLOWING)
      return;

    units::QTime now = pros::millis() * units::millisecond;
    path_planning::Output output = currentFollower->update(now - startTime, Odometry::instance->getPosition());

    util::DriveSignal velo(output.left_velocity_.getValue(), output.right_velocity_.getValue());
    util::DriveSignal feed(output.left_feedforward_voltage_.getValue() / 12.0, output.right_feedforward_voltage_.getValue() / 12.0);
    left_accel = output.left_accel_.getValue();
    right_accel = output.right_accel_.getValue();
    setVelocity(velo, feed);

  }
  void Drive::updateOutputs() {
    if(currentState == ControlState::OPEN_LOOP) {
      frontLeft->move_velocity(left_demand);
      backLeft->move_velocity(left_demand);
      frontRight->move_velocity(right_demand);
      backRight->move_velocity(right_demand);
    }
    else {
      double leftScaled = left_demand * 9.5493;
      leftScaled += left_feed_forward;

      double rightScaled = right_demand * 9.5493;
      rightScaled += right_feed_forward;


      printf("setting velo %f %f \n", leftScaled, rightScaled);
      frontLeft->move_velocity(leftScaled);
      backLeft->move_velocity(leftScaled);
      frontRight->move_velocity(rightScaled);
      backRight->move_velocity(rightScaled);
    }
  }
  void Drive::setBrakeMode(bool set) {
    frontLeft->set_brake_mode(set ? pros::E_MOTOR_BRAKE_BRAKE : pros::E_MOTOR_BRAKE_COAST);
    frontRight->set_brake_mode(set ? pros::E_MOTOR_BRAKE_BRAKE : pros::E_MOTOR_BRAKE_COAST);
    backRight->set_brake_mode(set ? pros::E_MOTOR_BRAKE_BRAKE : pros::E_MOTOR_BRAKE_COAST);
    backLeft->set_brake_mode(set ? pros::E_MOTOR_BRAKE_BRAKE : pros::E_MOTOR_BRAKE_COAST);
  }

  ControlState Drive::getState() {
    return currentState;
  }

  void Drive::setTrajectory(trajectory::TrajectoryIterator<trajectory::TimedState<geometry::Pose2dWithCurvature>> trajectory) {
    currentFollower = new path_planning::PathFollower(trajectory, path_planning::FollowerType::FEEDFORWARD_ONLY);
    currentState = ControlState::PATH_FOLLOWING;
    startTime = pros::millis() * units::millisecond;
  }

  bool Drive::isDoneWithTrajectory() {
    if(forceStopTrajectory_) {
      forceStopTrajectory_ = false;
      return true;
    }
    return currentFollower->isDone();
  }
  void Drive::overrideTrajectory() {
    forceStopTrajectory_= true;
  }


}
