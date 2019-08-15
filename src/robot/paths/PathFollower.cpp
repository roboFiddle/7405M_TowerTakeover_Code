//
// Created by alexweiss on 8/15/19.
//

#include "PathFollower.hpp"
#include "../Constants.hpp"
#include "PathConstants.hpp"
#include <cmath>

namespace path_planning {
  Output::Output() {
    Output(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  }
  Output::Output(units::QAngularSpeed left_velocity,
                 units::QAngularSpeed right_velocity,
                 units::QAngularAcceleration left_accel,
                 units::QAngularAcceleration right_accel,
                 units::Number left_feedforward_voltage,
                 units::Number right_feedforward_voltage) {
    left_velocity_ = left_velocity;
    right_velocity_ = right_velocity;
    left_accel_ = left_accel;
    right_accel_ = right_accel;
    left_feedforward_voltage_ = left_feedforward_voltage;
    right_feedforward_voltage_ = right_feedforward_voltage;

  }
  void Output::flip() {
    units::QAngularSpeed tmp_left_velocity = left_velocity_;
    left_velocity_ = -1*right_velocity_;
    right_velocity_ = -1*tmp_left_velocity;

    units::QAngularAcceleration tmp_left_accel = left_accel_;
    left_accel_ = -1*right_accel_;
    right_accel_ = -1*tmp_left_accel;

    units::Number tmp_left_feedforward = left_feedforward_voltage_;
    left_feedforward_voltage_ = -1*right_feedforward_voltage_;
    right_feedforward_voltage_ = -1*tmp_left_feedforward;
  }

  PathFollower::PathFollower(trajectory::TrajectoryIterator<trajectory::TimedState<geometry::Pose2dWithCurvature>> traj, FollowerType follower_type) :
  current_trajectory_(traj) {
    physics::DCMotorTransmission transmission(
        constants::RobotConstants::kDriveSpeedPerVolt,
        constants::RobotConstants::kDriveTorquePerVolt,
        constants::RobotConstants::kDriveFrictionVoltage);
    model_ = new physics::DifferentialDrive(
        constants::RobotConstants::kRobotMass,
        constants::RobotConstants::kRobotMoment,
        constants::RobotConstants::kRobotAngularDrag,
        constants::RobotConstants::kDriveWheelRadius,
        constants::RobotConstants::kDriveWheelTrackWidth / 2.0 * constants::RobotConstants::kTrackScrubFactor,
        transmission, transmission
    );
    dt_ = 0;
    last_time_ = INFINITY;
    mFollowerType_ = follower_type;
  }
  bool PathFollower::isDone() {
    return current_trajectory_.isDone();
  }
  geometry::Pose2d PathFollower::error() {
    return error_;
  }
  trajectory::TimedState<geometry::Pose2dWithCurvature> PathFollower::setpoint() {
    return setpoint_;
  }
  Output PathFollower::update(units::QTime timestamp, geometry::Pose2d current_state) {
    if (current_trajectory_.getProgress() == 0.0 && std::isinf(last_time_.getValue())) {
      last_time_ = timestamp;
    }

    dt_ = timestamp - last_time_;
    last_time_ = timestamp;
    trajectory::TrajectorySamplePoint<trajectory::TimedState<geometry::Pose2dWithCurvature>> sample_point = current_trajectory_.advance(dt_.getValue());
    setpoint_ = sample_point.state();
    if(!current_trajectory_.isDone()) {
      units::QSpeed velocity_m = setpoint_.velocity();
      units::QAcceleration acceleration_m = setpoint_.acceleration();
      units::QCurvature curvature_m = setpoint_.state().curvature();
      units::QDCurvatureDs dcurvature_ds_m = setpoint_.state().dcurvature();
      physics::DifferentialDrive::DriveDynamics dynamics = model_->solveInverseDynamics(
          physics::DifferentialDrive::ChassisState<units::QSpeed, units::QAngularSpeed>(velocity_m, velocity_m * curvature_m),
          physics::DifferentialDrive::ChassisState<units::QAcceleration, units::QAngularAcceleration>(acceleration_m,
                                             acceleration_m * curvature_m + velocity_m * velocity_m * dcurvature_ds_m));
      error_ = current_state.inverse().transformBy(setpoint_.state().pose());

      if (mFollowerType_ == FollowerType::FEEDFORWARD_ONLY) {
        current_output_ = Output(dynamics.wheel_velocity.left_, dynamics.wheel_velocity.right_,
            dynamics.wheel_acceleration.left_, dynamics.wheel_acceleration.right_,
            dynamics.voltage.left_, dynamics.voltage.right_);
      } else if (mFollowerType_ == FollowerType::PURE_PURSUIT) {
        current_output_ = updatePurePursuit(dynamics, current_state);
      } else if (mFollowerType_ == FollowerType::PID) {
        current_output_ = updatePID(dynamics, current_state);
      } else if (mFollowerType_ == FollowerType::NONLINEAR_FEEDBACK) {
        current_output_ = updateNonlinearFeedback(dynamics, current_state);
      }
    }
    else {
      current_output_ = Output();
    }
    return current_output_;

  }
  Output PathFollower::updatePID(physics::DifferentialDrive::DriveDynamics dynamics, geometry::Pose2d current_state) {

  }
  Output PathFollower::updatePurePursuit(physics::DifferentialDrive::DriveDynamics dynamics, geometry::Pose2d current_state) {

  }
  Output PathFollower::updateNonlinearFeedback(physics::DifferentialDrive::DriveDynamics dynamics, geometry::Pose2d current_state) {

  }
}