//
// Created by alexweiss on 8/14/19.
//

#include "Drive.hpp"
#include "Odometry.hpp"
#include "Inertial.hpp"
#include "../Constants.hpp"
#include <stdio.h>
#include <memory>

namespace subsystems {
  Drive::DriveManager Drive::instance;

  Drive::Drive() {
    frontLeft = new pros::Motor(constants::RobotConstants::motor_drive_frontleft, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    frontRight = new pros::Motor(constants::RobotConstants::motor_drive_frontright, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
    backLeft = new pros::Motor(constants::RobotConstants::motor_drive_backleft, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    backRight = new pros::Motor(constants::RobotConstants::motor_drive_backright, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
    //setBrakeMode(true);
    setOpenLoop(util::DriveSignal::NEUTRAL);
    forceStopTrajectory_ = false;
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
    currentState = ControlState::OPEN_LOOP;
    setBrakeMode(false);
    left_demand = signal.left();
    right_demand = signal.right();
  }
  void Drive::setVoltage(util::DriveSignal signal) {
    currentState = ControlState::VOLTAGE;
    setBrakeMode(false);
    left_demand = signal.left();
    right_demand = signal.right();
  }
  void Drive::setFromMacro(util::DriveSignal signal) {
    if (currentState != ControlState::POSITION_CONTROL) {
      currentState = ControlState::POSITION_CONTROL;
      setBrakeMode(true);
    }

    left_demand = signal.left();
    right_demand = signal.right();
  }
  void Drive::setVelocity(util::DriveSignal velocity, util::DriveSignal feedforward) {
    if (currentState != ControlState::PATH_FOLLOWING) {
      currentState = ControlState::PATH_FOLLOWING;
    }
    setBrakeMode(true);

    left_demand = velocity.left();
    right_demand = velocity.right();
    left_feed_forward = feedforward.left();
    right_feed_forward = feedforward.right();
  }
  void Drive::registerEnabledLoops(loops::Looper* enabledLooper){
    loops::Loop* thisLoop = new loops::Loop();
    thisLoop->onStart = []() {};
    thisLoop->onLoop = []() {
      switch (Drive::instance->getState()) {
        case ControlState::OPEN_LOOP :
          break;
        case ControlState::TURN_FOLLOWING:
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
    if(currentState == ControlState::OPEN_LOOP || currentState == ControlState::POSITION_CONTROL) {
      frontLeft->move_velocity(left_demand);
      backLeft->move_velocity(left_demand);
      frontRight->move_velocity(right_demand);
      backRight->move_velocity(right_demand);
    }
    else if(currentState == ControlState::VOLTAGE) {
      frontLeft->move_voltage(left_demand);
      backLeft->move_voltage(left_demand);
      frontRight->move_voltage(right_demand);
      backRight->move_voltage(right_demand);
    }
    else if(currentState == ControlState::PATH_FOLLOWING) {
      double leftScaled = left_demand * 9.5493 * 0.8; // 9.5493 = rad/s to RPM (60/2pi)
      //leftScaled += left_feed_forward;

      double rightScaled = right_demand * 9.5493 * (.8);
      //rightScaled += right_feed_forward;


      printf("setting velo %f %f \n\n", leftScaled, rightScaled);
      frontRight->move_velocity(rightScaled);
      frontLeft->move_velocity(leftScaled);
      backLeft->move_velocity(leftScaled);
      backRight->move_velocity(rightScaled);
    }
    else if(currentState == ControlState::TURN_FOLLOWING || currentState == ControlState::INTERIAL_TURN) {
      units::Angle currentHeading;
      if(currentState == ControlState::TURN_FOLLOWING)
        currentHeading = Odometry::instance->getPosition().rotation().getRadians() * units::radian;
      else
        currentHeading = Inertial::instance->getRotation();
      /* if(goalAngle.getValue() < 0)
        currentHeading = currentHeading - 360*units::degree; */
      units::Angle error = goalAngle - currentHeading;
      units::Angle deltaError = error - lastTurnError;
      totalTurnError += error;
      double velo = constants::RobotConstants::turnKP * error.getValue();
      if(fast_turn_)
        velo *= 2;
      //velo += constants::RobotConstants::turnKI * totalTurnError.getValue();
      //velo += constants::RobotConstants::turnKD * deltaError.getValue();

      frontLeft->move_velocity(-velo);
      backLeft->move_velocity(-velo);
      frontRight->move_velocity(velo);
      backRight->move_velocity(velo);
      printf("ROT FUCK %f \n",Odometry::instance->getPosition().rotation().getDegrees());
      printf("TURN %f %f %f\n", std::fabs(goalAngle.getValue() - currentHeading.getValue()), Odometry::instance->getPosition().rotation().getRadians(), velo);
      if(std::fabs(goalAngle.getValue() - currentHeading.getValue()) < 0.2*(fast_turn_ ? 2 : 1) || (stop_ && std::fabs(velo) < 15))
        turnFinishCount++;
      else
        turnFinishCount = 0;
      lastTurnError = error;
    }
    else if(currentState == ControlState::TURN_BACK_WHEEL) {
      setBrakeMode(true);
      double delta = goalAngle.getValue() + (double) backLeft->get_position();
      printf("TURN WHEEL %f %f\n", delta, orgDel);
      int sign = orgDel > 0.0 ? -1 : 1;
      if(std::fabs(delta) > std::fabs(orgDel * 0.3)) {
          frontRight->move_voltage(sign * -7000);
          backRight->move_voltage(sign * -7000);
          backLeft->move_voltage(sign * 7000);
          frontLeft->move_voltage(sign * 7000);
      }
      else if(delta*(-sign) > 0.0)  {
          frontRight->move_voltage(sign * -2000);
          backRight->move_voltage(sign * -2000);
          backLeft->move_voltage(sign * 2000);
          frontLeft->move_voltage(sign * 2000);
      }
      else {
        frontRight->move_velocity(sign * 500);
        backRight->move_velocity(sign * 500);
        backLeft->move_velocity(sign * -500);
        frontLeft->move_velocity(sign * -500);
        turnFinishCount += 2;
      }
    }
    else if(currentState == ControlState::ENCODER_WHEEL) {
      setBrakeMode(true);
      double delta = goalAngle.getValue() - (double) backLeft->get_position();
      int sign = orgDel > 0.0 ? 1 : -1;
      sign *= (fast_turn_ ? 2 : 1);
      if(std::fabs(delta) > std::fabs(orgDel * 0.3)) {
          frontRight->move_voltage(sign * 7000);
          backRight->move_voltage(sign * 7000);
          backLeft->move_voltage(sign * 7000);
          frontLeft->move_voltage(sign * 7000);
      }
      else if(delta*sign > 0.0)  {
          frontRight->move_voltage(sign * 3500);
          backRight->move_voltage(sign * 3500);
          backLeft->move_voltage(sign * 3500);
          frontLeft->move_voltage(sign * 3500);
      }
      else {
        frontRight->move_velocity(sign * -500);
        backRight->move_velocity(sign * -500);
        backLeft->move_velocity(sign * -500);
        frontLeft->move_velocity(sign * -500);
        turnFinishCount += 5;
      }
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

  void Drive::setTrajectory(trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> traj) {
    currentState = ControlState::PATH_FOLLOWING;
    currentTrajectory = traj;

    currentTimedView = new trajectory::TimedView(&currentTrajectory);
    std::shared_ptr<trajectory::TimedView<geometry::Pose2dWithCurvature>> ptr(currentTimedView);
    trajectory::TrajectoryIterator<trajectory::TimedState<geometry::Pose2dWithCurvature>> iterator(ptr);
    currentFollower = new path_planning::PathFollower(iterator, path_planning::FollowerType::FEEDFORWARD_ONLY);
    startTime = pros::millis() * units::millisecond;
  }
  void Drive::setTurn(units::Angle heading, bool speed) {
    currentState = ControlState::TURN_FOLLOWING;
    subsystems::Odometry::instance->resetPosition();
    goalAngle = heading;
    lastTurnError = 0.0;
    totalTurnError = 0.0;
    turnFinishCount = 0;
    setBrakeMode(true);
    fast_turn_ = speed;
  }
  void Drive::setInertialTurn(units::Angle heading, bool speed, bool reset, bool s) {
    currentState = ControlState::INTERIAL_TURN;
    if(reset)
      subsystems::Inertial::instance->resetRotation();
    goalAngle = heading;
    lastTurnError = 0.0;
    totalTurnError = 0.0;
    turnFinishCount = 0;
    setBrakeMode(true);
    fast_turn_ = speed;
    stop_ = s;
  }
  void Drive::setTurnWheel(units::Angle heading) {
    currentState = ControlState::TURN_BACK_WHEEL;
    clicksWheel = heading.getValue() * constants::RobotConstants::BACK_WHEELBASE_RADIUS  / 12.56 * 180 / 3.1415;
    goalAngle = clicksWheel - backLeft->get_position();
    orgDel = clicksWheel;
    turnFinishCount = 0;
    setBrakeMode(true);
  }
  void Drive::setEncoderWheel(units::QLength dist, bool fast) {
    currentState = ControlState::ENCODER_WHEEL;
    clicksWheel = dist.Convert(units::inch) / 12.56 * 360;
    goalAngle = clicksWheel + backLeft->get_position();
    orgDel = clicksWheel;
    turnFinishCount = 0;
    setBrakeMode(true);
    fast_turn_ = fast;
  }
  bool Drive::isDoneWithTrajectory() {
    if(forceStopTrajectory_) {
      forceStopTrajectory_ = false;
      return true;
    }
    if(currentState == ControlState::TURN_FOLLOWING || currentState == ControlState::INTERIAL_TURN || currentState == ControlState::TURN_BACK_WHEEL || currentState == ControlState::ENCODER_WHEEL) {
      return turnFinishCount > 10;
    }
    return currentFollower->isDone();
  }
  void Drive::overrideTrajectory() {
    forceStopTrajectory_= true;
  }


}
