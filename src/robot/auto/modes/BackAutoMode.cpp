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
#include "../actions/DriveInertialTurnAction.hpp"
#include "../actions/DriveTurnWheelAction.hpp"
#include "../actions/ResetLiftTrayPosition.hpp"
#include "../actions/WaitForCubeInIntakeAction.hpp"
#include "../../subsystems/Odometry.hpp"

namespace auton {
  void BackAutoMode::routine() {
    flipOut();
    //runAction(new actions::WaitAction(0.5));
    subsystems::Odometry::instance->setCurrentPosition(9 * units::inch, 50.4 * units::inch, 0);
    std::list<actions::Action*> drives;
    if(true) {
      drives.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("backLineForward").get(flip_)));
      drives.push_back(new actions::DriveTrajectory(trajectory::TimingUtil::reverseTimed(path_planning::TrajectorySet::instance->get("backBackForTowerCube").get(flip_))));
      drives.push_back(new actions::DriveInertialTurnAction((flip_ ? 34 : -34) * units::degree - subsystems::Odometry::instance->getPosition().rotation().getAngle() + 90*units::degree, true));
      drives.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("backGetSecondCube").get(flip_)));
      drives.push_back(new actions::DriveTrajectory(trajectory::TimingUtil::reverseTimed(path_planning::TrajectorySet::instance->get("backOffTower").get(flip_))));
      //drives.push_back(new actions::OpenLoopDriveAction(util::DriveSignal(0, 0), 0.375));
      drives.push_back(new actions::DriveInertialTurnAction((flip_ ? 118 : -120) * units::degree, false, true));
      //drives.push_back(new actions::OpenLoopDriveAction(util::DriveSignal(0, 0), 0.375));
      drives.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get(flip_ ? "backLongAlign" : "backLongAlign").get(flip_)));
    }
    else {
      drives.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("backLineForward").get(flip_)));
      drives.push_back(new actions::DriveTrajectory(trajectory::TimingUtil::reverseTimed(path_planning::TrajectorySet::instance->get("backLongAlign").get(flip_))));
      drives.push_back(new actions::OpenLoopDriveAction(util::DriveSignal(0, 0), 0.5));
      drives.push_back(new actions::DriveInertialTurnAction((flip_ ? -135 : -150) * units::degree, false, true));
      drives.push_back(new actions::OpenLoopDriveAction(util::DriveSignal(0, 0), 0.5));
      drives.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("backSetup").get(flip_)));
    }

    /*drives.push_back(new actions::OpenLoopDriveAction(util::DriveSignal(0,0), 0.25));
    drives.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("backAlign").get(flip_)));
    */

    std::list<actions::Action*> driveAndIntake;
    driveAndIntake.push_back(new actions::OpenLoopIntakeAction(200, 0));
    driveAndIntake.push_back(new actions::SeriesAction(drives));
    runAction(new actions::ParallelAction(driveAndIntake));

    //runAction(new actions::DriveTurnAction(-95 * units::degree * (flip_ ? -1 : 1), true));
    //runAction(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("backSetup").get(false)));

    runAction(new actions::TrayEnableStackAction(100, 32));

    //stacking stack
    std::list<actions::Action*> pullBackFromStack;
    pullBackFromStack.push_back(new actions::OpenLoopIntakeAction(-100, 0));
    pullBackFromStack.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("skillsStackPullBack").get(false)));
    runAction(new actions::ParallelAction(pullBackFromStack));

  }

}
