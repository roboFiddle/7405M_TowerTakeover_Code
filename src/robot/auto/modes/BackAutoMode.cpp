//
// Created by alexweiss on 10/20/19.
//

#include "BackAutoMode.hpp"
#include "../actions/DriveTrajectory.hpp"
#include "../actions/WaitAction.hpp"
#include "../actions/OpenLoopDriveAction.hpp"
#include "../actions/OpenLoopIntakeAction.hpp"
#include "../actions/ParallelAction.hpp"
#include "../actions/TrayEnableStackAction.hpp"
#include "../../paths/TrajectorySet.hpp"
#include "../actions/TrayPosition.hpp"
#include "../actions/OpenLoopLiftAction.hpp"
#include "../actions/ResetLiftTrayPosition.hpp"

namespace auton {
  void BackAutoMode::routine() {
    flipOut();

    std::list<actions::Action*> driveAndIntake;
    driveAndIntake.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("backForward").get(false)));
    driveAndIntake.push_back(new actions::TrayPosition(-50, false));
    driveAndIntake.push_back(new actions::OpenLoopIntakeAction(200, -1));
    runAction(new actions::ParallelAction(driveAndIntake));

    std::list<actions::Action*> driveAndIntake2;
    driveAndIntake2.push_back(new actions::DriveTrajectory(trajectory::TimingUtil::reverseTimed(path_planning::TrajectorySet::instance->get("backS").get(flip_))));
    driveAndIntake2.push_back(new actions::OpenLoopIntakeAction(100, 0));
    runAction(new actions::ParallelAction(driveAndIntake2));
    runAction(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("backSetup").get(flip_)));

    runAction(new actions::OpenLoopIntakeAction(-100, 0.8));
    runAction(new actions::TrayEnableStackAction());
  }

}
