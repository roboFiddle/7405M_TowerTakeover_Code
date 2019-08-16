//
// Created by alexweiss on 8/14/19.
//

#include "TrajectorySet.hpp"
#include "DriveMotionPlanner.hpp"
#include "main.h"

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
    generatorCalls();
    //pros::Task task(generatorCalls, (void*) this, TASK_PRIORITY_DEFAULT - 1, TASK_STACK_DEPTH_DEFAULT, "GENERATE TRAJECTORY TASK");
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
  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::getTestForwardTrajectory() {
    std::vector<geometry::Pose2d> waypoints;
    waypoints.push_back(geometry::Pose2d(0, 0, geometry::Rotation2d::fromDegrees(0)));
    waypoints.push_back(geometry::Pose2d(18 * units::inch, 0, geometry::Rotation2d::fromDegrees(0)));

    std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> noConstraints;

    return DriveMotionPlanner::generateTrajectory(false, waypoints, noConstraints, 0.2 * units::mps, 0.4 * units::mps2, 8.0);
  }
  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::getTestSCurveTrajecory() {
    std::vector<geometry::Pose2d> waypoints;
    waypoints.push_back(geometry::Pose2d(0, 0, geometry::Rotation2d::fromDegrees(0)));
    waypoints.push_back(geometry::Pose2d(10 * units::inch, 10 * units::inch, geometry::Rotation2d::fromDegrees(0)));
    /* waypoints.push_back(geometry::Pose2d(0 * units::inch, 20 * units::inch, geometry::Rotation2d::fromDegrees(180)));
    waypoints.push_back(geometry::Pose2d(-10 * units::inch, 30 * units::inch, geometry::Rotation2d::fromDegrees(90)));
    waypoints.push_back(geometry::Pose2d(0 * units::inch, 40 * units::inch, geometry::Rotation2d::fromDegrees(0))); */

    std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> noConstraints;

    return DriveMotionPlanner::generateTrajectory(false, waypoints, noConstraints, 0.2 * units::mps, 0.4 * units::mps2, 8.0);
  }
  void TrajectorySet::generatorCalls() {
    //addToMap("testForward", MirroredTrajectory(getTestForwardTrajectory()));
    //addToMap("testSCurve", MirroredTrajectory(getTestSCurveTrajecory()));
    complete_ = true;
  }
}