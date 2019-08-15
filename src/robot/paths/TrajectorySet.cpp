//
// Created by alexweiss on 8/14/19.
//

#include "TrajectorySet.hpp"
#include "main.h"

namespace path_planning {
  MirroredTrajectory::MirroredTrajectory(trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> right) {
    right_ = right;
    left_ = trajectory::TrajectoryUtil::mirrorTimed(right);
  }
  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> MirroredTrajectory::get(bool left) {
    if(left)
      return left_;
    return right_;
  }

  TrajectorySet::TrajectorySet() {
    complete_ = false;
    pros::Task task(generatorCalls, (void*) this, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "GENERATE TRAJECTORY TASK");
  }
  bool TrajectorySet::isDoneGenerating() {
    return complete_;
  }
  void TrajectorySet::addToMap(std::string name, trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> traj) {
    trajectories_.insert({name, traj});
  }
  bool TrajectorySet::inSet(std::string name) {
    return trajectories_.count(name);
  }
  MirroredTrajectory TrajectorySet::get(std::string name) {
    return trajectories_.at(name);
  }
  void TrajectorySet::generatorCalls(void* param) {
    TrajectorySet* instance = static_cast<TrajectorySet*>(param);

    instance->complete_ = true;
  }
}