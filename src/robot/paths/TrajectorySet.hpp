//
// Created by alexweiss on 8/14/19.
//

#ifndef INC_7405M_CODE_SRC_ROBOT_PATHS_TRAJECTORYSET_HPP_
#define INC_7405M_CODE_SRC_ROBOT_PATHS_TRAJECTORYSET_HPP_

#include "../../lib/meecan_lib.hpp"

namespace path_planning {

  class MirroredTrajectory {
   public:
    trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> backward_;
    trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> forward_;

    MirroredTrajectory(trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> right);
    trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> get(bool left);
  };

  class TrajectorySet {
   private:
    std::map<std::string, MirroredTrajectory> trajectories_;
    static trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> getBackForward();
    static trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> getBackS();
    static trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> getFrontSetup();
    static trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> getBackSetup();
    static trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> getStackPullBack();
    static trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> getPSkillsForward();
    static trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> getPSetup();
    static trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> getPFirstTower();
    static trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> getPBackFirstTower();

    static trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> getPurePursuitTest();


   public:
    bool complete_;

    TrajectorySet();
    void startGenerator();
    bool isDoneGenerating();
    bool inSet(std::string name);
    MirroredTrajectory get(std::string name);
    void addToMap(std::string name, MirroredTrajectory traj);
    void generatorCalls();
    struct TrajectorySetManager : public util::Singleton<TrajectorySet, TrajectorySetManager> {};
    static TrajectorySetManager instance;
  };
}

#endif //INC_7405M_CODE_SRC_ROBOT_PATHS_TRAJECTORYSET_HPP_
