//
// Created by alexweiss on 10/20/19.
//

#include "FrontAutoMode.hpp"
#include "../actions/DriveTrajectory.hpp"
#include "../actions/DriveMoveWheelAction.hpp"
#include "../actions/DriveInertialTurnAction.hpp"
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
  void FrontAutoMode::routine() {
    flipOut();

    std::list<actions::Action*> drives;
    drives.push_back(new actions::DriveMoveWheelAction(24 * units::inch));
    drives.push_back(new actions::WaitAction(0.25));
    drives.push_back(new actions::DriveInertialTurnAction(-35 * units::degree));
    drives.push_back(new actions::WaitAction(0.25));
    drives.push_back(new actions::DriveMoveWheelAction(12 * units::inch));
    drives.push_back(new actions::WaitAction(0.25));
    drives.push_back(new actions::DriveMoveWheelAction(-12 * units::inch));
    drives.push_back(new actions::WaitAction(0.25));
    drives.push_back(new actions::DriveInertialTurnAction(-80 * units::degree));
    drives.push_back(new actions::DriveMoveWheelAction(10 * units::inch));
    drives.push_back(new actions::WaitAction(0.25));
    drives.push_back(new actions::DriveInertialTurnAction(-130 * units::degree));
    drives.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("frontSetup").get(false)));

    std::list<actions::Action*> driveAndIntakeB;
    driveAndIntakeB.push_back(new actions::SeriesAction(drives));
    driveAndIntakeB.push_back(new actions::OpenLoopIntakeAction(200, 0));
    runAction(new actions::ParallelAction(driveAndIntakeB));

    runAction(new actions::TrayEnableStackAction(100, 35));

    //stacking stack
    std::list<actions::Action*> pullBackFromStack;
    pullBackFromStack.push_back(new actions::OpenLoopIntakeAction(-100, 0));
    pullBackFromStack.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("skillsStackPullBack").get(false)));
    runAction(new actions::ParallelAction(pullBackFromStack));
  }

}
