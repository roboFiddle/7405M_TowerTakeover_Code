//
// Created by alexweiss on 8/14/19.
//

#include "TrajectorySet.hpp"
#include "DriveMotionPlanner.hpp"
#include "PathConstants.hpp"

namespace path_planning {
  TrajectorySet::TrajectorySetManager TrajectorySet::instance;

  MirroredTrajectory::MirroredTrajectory(trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> forward) {
    forward_ = forward;
    backward_ = trajectory::TrajectoryUtil::mirrorTimed(forward);
  }
  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> MirroredTrajectory::get(bool back) {
    if(back)
      return backward_;
    return forward_;
  }

  TrajectorySet::TrajectorySet() {
    complete_ = false;
  }
  bool TrajectorySet::isDoneGenerating() {
    return complete_;
  }
  void TrajectorySet::addToMap(std::string name, MirroredTrajectory traj) {
    trajectories_.insert({name, traj});
  }
  bool TrajectorySet::inSet(std::string name) {
    return trajectories_.count(name);
  }
  MirroredTrajectory TrajectorySet::get(std::string name) {
    return trajectories_.at(name);
  }
  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::getBackForward() {
    std::vector<geometry::Pose2d> waypoints;
    waypoints.push_back(geometry::Pose2d(0, 0, geometry::Rotation2d::fromDegrees(0)));
    waypoints.push_back(geometry::Pose2d(32 * units::inch, 0, geometry::Rotation2d::fromDegrees(0)));

    std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> noConstraints;

    return DriveMotionPlanner::generateTrajectory(false, waypoints, noConstraints,
        constants::PathConstants::kMaxVelocity * 0.5, constants::PathConstants::kMaxAccel, 8.0);
  }
  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::getBackS() {
    std::vector<geometry::Pose2d> waypoints;

    waypoints.push_back(geometry::Pose2d(0 * units::inch, 0 * units::inch, geometry::Rotation2d::fromDegrees(0)));
    waypoints.push_back(geometry::Pose2d(8 * units::inch, 23 * units::inch, geometry::Rotation2d::fromDegrees(82)));

    std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> noConstraints;

    return DriveMotionPlanner::generateTrajectory(false, waypoints, noConstraints,
        constants::PathConstants::kMaxVelocity, constants::PathConstants::kMaxAccel, 8.0);
  }
  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::getFrontSetup() {
    std::vector<geometry::Pose2d> waypoints;

    waypoints.push_back(geometry::Pose2d(0 * units::inch, 0 * units::inch, geometry::Rotation2d::fromDegrees(0)));
    waypoints.push_back(geometry::Pose2d(17 * units::inch, -4 * units::inch, geometry::Rotation2d::fromDegrees(-45)));

    std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> noConstraints;

    return DriveMotionPlanner::generateTrajectory(false, waypoints, noConstraints,
        constants::PathConstants::kMaxVelocity, constants::PathConstants::kMaxAccel, 8.0);
  }
  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::getBackSetup() {
    std::vector<geometry::Pose2d> waypoints;

    waypoints.push_back(geometry::Pose2d(0 * units::inch, 0 * units::inch, geometry::Rotation2d::fromDegrees(0)));
    waypoints.push_back(geometry::Pose2d(19 * units::inch, -9 * units::inch, geometry::Rotation2d::fromDegrees(-45)));

    std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> noConstraints;

    return DriveMotionPlanner::generateTrajectory(false, waypoints, noConstraints,
        constants::PathConstants::kMaxVelocity, constants::PathConstants::kMaxAccel, 8.0);
  }

  void TrajectorySet::generatorCalls() {
    printf("start\n");
    addToMap("backForward", MirroredTrajectory(getBackForward()));
    addToMap("backS", MirroredTrajectory(getBackS()));
    addToMap("frontSetup", MirroredTrajectory(getFrontSetup()));
    addToMap("backSetup", MirroredTrajectory(getBackSetup()));
    complete_ = true;
    printf("end\n");
  }
}
