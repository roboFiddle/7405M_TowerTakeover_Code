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

namespace auton {
  void BackAutoMode::routine() {
    printf("started test mode\n");
    std::list<actions::Action*> driveAndIntake;
    driveAndIntake.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("backForward").get(false)));
    driveAndIntake.push_back(new actions::OpenLoopIntakeAction(200, 0));
    runAction(new actions::ParallelAction(driveAndIntake));

    std::list<actions::Action*> driveAndIntake2;
    driveAndIntake2.push_back(new actions::DriveTrajectory(trajectory::TimingUtil::reverseTimed(path_planning::TrajectorySet::instance->get("backS").get(false))));
    driveAndIntake2.push_back(new actions::OpenLoopIntakeAction(200, 1));
    runAction(new actions::ParallelAction(driveAndIntake2));
    runAction(new actions::WaitAction(0.5));
    runAction(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("backSetup").get(false)));
    runAction(new actions::OpenLoopDriveAction(util::DriveSignal(50, 50), 0.5));
    runAction(new actions::OpenLoopIntakeAction(-100, 1));
    runAction(new actions::TrayEnableStackAction());
    //pros::lcd::print(3, "done with stack");
    //runAction(new actions::DriveTrajectory(trajectory::TimingUtil::reverseTimed(path_planning::TrajectorySet::instance->get("backForward").get(false))));
  }

}
