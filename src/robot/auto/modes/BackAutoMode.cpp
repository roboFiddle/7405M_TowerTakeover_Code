//
// Created by alexweiss on 10/20/19.
//

#include "BackAutoMode.hpp"
#include "../actions/DriveTrajectory.hpp"
#include "../actions/WaitAction.hpp"
#include "../actions/OpenLoopDriveAction.hpp"
#include "../actions/OpenLoopIntakeAction.hpp"
#include "../actions/ParallelAction.hpp"
#include "../actions/SeriesAction.hpp"
#include "../actions/TrayEnableStackAction.hpp"
#include "../../paths/TrajectorySet.hpp"
#include "../actions/TrayPosition.hpp"
#include "../actions/OpenLoopLiftAction.hpp"
#include "../actions/DriveTurnAction.hpp"
#include "../actions/ResetLiftTrayPosition.hpp"

namespace auton {
  void BackAutoMode::routine() {

    flipOut();
    //runAction(new actions::OpenLoopDriveAction(util::DriveSignal(0.0, 0.0), 0));
    pros::Task::delay(20);
    std::list<actions::Action*> driveAndIntake;
    driveAndIntake.push_back(new actions::OpenLoopIntakeAction(200, 0));
    driveAndIntake.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("backJForward").get(false)));
    runAction(new actions::ParallelAction(driveAndIntake));

    std::list<actions::Action*> driveAndIntake2;
    driveAndIntake2.push_back(new actions::OpenLoopIntakeAction(200, 0));
    driveAndIntake2.push_back(new actions::DriveTurnAction(34 * units::degree * (flip_ ? -1 : 1)));
    runAction(new actions::ParallelAction(driveAndIntake2));

    std::list<actions::Action*> driveAndIntakeA;
    driveAndIntakeA.push_back(new actions::OpenLoopIntakeAction(200, 0));
    driveAndIntakeA.push_back(new actions::DriveTrajectory(trajectory::TimingUtil::reverseTimed(path_planning::TrajectorySet::instance->get("backToLine").get(false))));
    runAction(new actions::ParallelAction(driveAndIntakeA));

    runAction(new actions::OpenLoopDriveAction(util::DriveSignal(-100, -100), 0.5));

    std::list<actions::Action*> driveAndIntakeX;
    driveAndIntakeX.push_back(new actions::OpenLoopIntakeAction(600, 0));
    driveAndIntakeX.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("backLineForward").get(false)));
    runAction(new actions::ParallelAction(driveAndIntakeX));

    std::list<actions::Action*> driveAndIntake3;
    driveAndIntake3.push_back(new actions::OpenLoopIntakeAction(200, 0));
    driveAndIntake3.push_back(new actions::DriveTrajectory(trajectory::TimingUtil::reverseTimed(path_planning::TrajectorySet::instance->get("backAlign").get(false))));
    runAction(new actions::ParallelAction(driveAndIntake3));

    runAction(new actions::DriveTurnAction(-45 * units::degree * (flip_ ? -1 : 1)));

    std::list<actions::Action*> driveAndIntake4;
    driveAndIntake4.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("backSetup").get(flip_)));
    driveAndIntake4.push_back(new actions::OpenLoopIntakeAction(200, 0));
    runAction(new actions::ParallelAction(driveAndIntake4));

    runAction(new actions::OpenLoopIntakeAction(-80, 0.75));
    runAction(new actions::TrayEnableStackAction(1.15));
    runAction(new actions::WaitAction(0.3));

    std::list<actions::Action*> pullBackFromStack;
    pullBackFromStack.push_back(new actions::OpenLoopIntakeAction(-100, 0));
    pullBackFromStack.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("stackPullBack").get(flip_)));
    runAction(new actions::ParallelAction(pullBackFromStack));

  }

}
