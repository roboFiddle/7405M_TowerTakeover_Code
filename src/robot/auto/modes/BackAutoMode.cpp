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
    runAction(new actions::OpenLoopDriveAction(util::DriveSignal(200, 200), 0.3));
    runAction(new actions::OpenLoopDriveAction(util::DriveSignal(-200, -200), 0.5));
    runAction(new actions::OpenLoopDriveAction(util::DriveSignal(0, 0), 0));
    runAction(new actions::OpenLoopIntakeAction(200, 1));
    std::list<actions::Action*> TrayAndOuttakeRelease;
    TrayAndOuttakeRelease.push_back(new actions::TrayPosition(1000, false));
    TrayAndOuttakeRelease.push_back(new actions::OpenLoopIntakeAction(-200, 0));
    runAction(new actions::ParallelAction(TrayAndOuttakeRelease));

    std::list<actions::Action*> TrayAndLiftDown;
    TrayAndLiftDown.push_back(new actions::OpenLoopLiftAction(50, 0));
    TrayAndLiftDown.push_back(new actions::TrayPosition(300, false));
    runAction(new actions::ParallelAction(TrayAndLiftDown));
    runAction(new actions::ResetLiftTrayPosition());


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
