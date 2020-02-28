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
#include "../actions/OpenLoopTrayAction.hpp"
#include "../actions/DriveTurnAction.hpp"
#include "../actions/DriveTurnWheelAction.hpp"
#include "../actions/ResetLiftTrayPosition.hpp"
#include "../actions/WaitForCubeInIntakeAction.hpp"
#include "../../subsystems/Odometry.hpp"

namespace auton {
  void BackAutoMode::routine() {
    //flipOut();
    subsystems::Odometry::instance->setCurrentPosition(9 * units::inch, 50.4 * units::inch, 0);

    std::list<actions::Action*> drives;
    drives.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("backLineForward").get(flip_)));
    drives.push_back(new actions::DriveTrajectory(trajectory::TimingUtil::reverseTimed(path_planning::TrajectorySet::instance->get("backBackForTowerCube").get(flip_))));
    drives.push_back(new actions::DriveTurnAction(30 * units::degree));
    drives.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("backGetTowerCube").get(flip_)));
    drives.push_back(new actions::DriveTrajectory(trajectory::TimingUtil::reverseTimed(path_planning::TrajectorySet::instance->get("backGetTowerCube").get(flip_))));
    drives.push_back(new actions::DriveTurnWheelAction(140 * units::degree));
    drives.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("backAlign").get(flip_)));
    /*drives.push_back(new actions::OpenLoopDriveAction(util::DriveSignal(0,0), 0.25));
    drives.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("backAlign").get(flip_)));
    */

    std::list<actions::Action*> driveAndIntake;
    driveAndIntake.push_back(new actions::OpenLoopIntakeAction(200, 0));
    driveAndIntake.push_back(new actions::SeriesAction(drives));
    runAction(new actions::ParallelAction(driveAndIntake));

    //runAction(new actions::DriveTurnAction(-95 * units::degree * (flip_ ? -1 : 1), true));
    //runAction(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("backSetup").get(false)));

    runAction(new actions::TrayEnableStackAction(95));
    runAction(new actions::OpenLoopIntakeAction(-100, 0.65));

    //stacking stack
    std::list<actions::Action*> pullBackFromStack;
    pullBackFromStack.push_back(new actions::OpenLoopIntakeAction(-100, 0));
    pullBackFromStack.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("skillsStackPullBack").get(false)));
    runAction(new actions::ParallelAction(pullBackFromStack));

  }

}
