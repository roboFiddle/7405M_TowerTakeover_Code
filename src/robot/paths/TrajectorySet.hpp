//
// Created by alexweiss on 8/14/19.
//

#ifndef INC_7405M_CODE_SRC_ROBOT_PATHS_TRAJECTORYSET_HPP_
#define INC_7405M_CODE_SRC_ROBOT_PATHS_TRAJECTORYSET_HPP_

#include "../../lib/meecan_lib.hpp"

namespace path_planning {

  class MirroredTrajectory {
   public:
    trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> left_;
    trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> right_;

    MirroredTrajectory(trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> right);
    trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> get(bool left);
  };

  class TrajectorySet {
   private:
    std::map<std::string, MirroredTrajectory> trajectories;
    void addToMap(std::string name, trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> traj);
    trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> emptyTrajectory();
    bool complete_;
    void generatorCalls();
   public:
    TrajectorySet();
    void generate();
    bool isDoneGenerating();

    struct TrajectorySetManager : public util::Singleton<TrajectorySet, TrajectorySetManager> {};
    static TrajectorySetManager instance;
  };
}

#endif //INC_7405M_CODE_SRC_ROBOT_PATHS_TRAJECTORYSET_HPP_
