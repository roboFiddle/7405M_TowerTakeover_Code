//
// Created by alexweiss on 8/14/19.
//

#include "DriveMotionPlanner.hpp"
#include "../Constants.hpp"
#include "PathConstants.hpp"

namespace path_planning {
  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> DriveMotionPlanner::generateTrajectory(
      bool reversed,
      std::vector<geometry::Pose2d> waypoints,
      std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> constraints,
      units::QSpeed max_vel,  // inches/s
      units::QAcceleration max_accel,  // inches/s^2
      units::Number max_voltage) {
    generateTrajectory(reversed, waypoints, constraints, 0.0, 0.0, max_vel, max_accel, max_voltage);
  }

  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> DriveMotionPlanner::generateTrajectory(
      bool reversed,
      std::vector<geometry::Pose2d> waypoints,
      std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> constraints,
      units::QSpeed start_vel,
      units::QSpeed end_vel,
      units::QSpeed max_vel,  // inches/s
      units::QAcceleration max_accel,  // inches/s^2
      units::Number max_voltage)  {

    physics::DCMotorTransmission transmission(
        constants::RobotConstants::kDriveSpeedPerVolt,
        constants::RobotConstants::kDriveTorquePerVolt,
        constants::RobotConstants::kDriveFrictionVoltage);
    physics::DifferentialDrive drive_model(
        constants::RobotConstants::kRobotMass,
        constants::RobotConstants::kRobotMoment,
        constants::RobotConstants::kRobotAngularDrag,
        constants::RobotConstants::kDriveWheelRadius,
        constants::RobotConstants::kDriveWheelTrackWidth / 2.0 * constants::RobotConstants::kTrackScrubFactor,
        transmission, transmission
    );

    std::vector<geometry::Pose2d> waypoints_maybe_flipped = waypoints;
    geometry::Pose2d flip = geometry::Pose2d::fromRotation(geometry::Rotation2d(-1, 0));
    // TODO re-architect the spline generator to support reverse.
    if (reversed) {
      waypoints_maybe_flipped.clear();
      for (int i = 0; i < waypoints.size(); ++i) {
        waypoints_maybe_flipped.push_back(waypoints.at(i).transformBy(flip));
      }
    }

    trajectory::Trajectory<geometry::Pose2dWithCurvature> traj = trajectory::TrajectoryUtil::trajectoryFromSplineWaypoints(
        waypoints_maybe_flipped, constants::PathConstants::kMaxDx, constants::PathConstants::kMaxDy, constants::PathConstants::kMaxDtheta);

    if (reversed) {
      std::vector<geometry::Pose2dWithCurvature> flipped;
      for (int i = 0; i < traj.length(); ++i) {
        flipped.push_back(geometry::Pose2dWithCurvature(traj.getState(i).pose().transformBy(flip), -1*traj.getState(i).curvature(), traj.getState(i).dcurvature()));
      }
      traj = trajectory::Trajectory<geometry::Pose2dWithCurvature>(flipped);
    }


    trajectory::DifferentialDriveDynamicsConstraint<geometry::Pose2dWithCurvature> drive_constraint(&drive_model, max_voltage);
    std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature> *> constraints_list;
    constraints_list.push_back(&drive_constraint);
    for(trajectory::TimingConstraint<geometry::Pose2dWithCurvature> * constraint : constraints) {
      constraints_list.push_back(constraint);
    }

    // Generate the timed trajectory.
    trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>>
        timed_trajectory = trajectory::TimingUtil::timeParameterizeTrajectory(
        reversed,
        trajectory::DistanceView<geometry::Pose2dWithCurvature>(&traj),
        constants::PathConstants::kMaxDx, constraints_list, start_vel, end_vel, max_vel, max_accel);
    return timed_trajectory;
  }
}