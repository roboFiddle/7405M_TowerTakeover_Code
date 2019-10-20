//
// Created by alexweiss on 8/15/19.
//

#include "DriveTrajectory.hpp"
#include "../../subsystems/Drive.hpp"

namespace auton {
  namespace actions {
    DriveTrajectory::DriveTrajectory(trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> trajectory)
    : DriveTrajectory( trajectory, false) {}
    DriveTrajectory::DriveTrajectory(trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> trajectory, bool reset_pose) :
    trajectory_(trajectory), reset_pose_(reset_pose ) {}
    bool DriveTrajectory::isFinished() {
      return subsystems::Drive::instance->isDoneWithTrajectory();
    }
    void DriveTrajectory::start() {
      subsystems::Drive::instance->setTrajectory(trajectory_);
    }
    void DriveTrajectory::update() {

    }
    void DriveTrajectory::done() {

    }
  }
}