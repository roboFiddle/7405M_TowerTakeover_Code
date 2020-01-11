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
    bool six_cube_ = true;
    if(six_cube_) {
      runAction(new actions::OpenLoopDriveAction(util::DriveSignal(100.0, 100.0), 0.35));
      runAction(new actions::OpenLoopDriveAction(util::DriveSignal(-200.0, -200.0), 0.5));
    }

    flipOut();
    runAction(new actions::OpenLoopDriveAction(util::DriveSignal(0.0, 0.0), 0));

    if(!six_cube_) {
      std::list<actions::Action*> driveAndIntake;
      driveAndIntake.push_back(new actions::OpenLoopIntakeAction(200, 0));
      driveAndIntake.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("backForward").get(false)));
      runAction(new actions::ParallelAction(driveAndIntake));

      std::list<actions::Action*> driveAndIntake2;
      driveAndIntake2.push_back(new actions::OpenLoopIntakeAction(200, 0));
      driveAndIntake2.push_back(new actions::DriveTurnAction(26 * units::degree * (flip_ ? -1 : 1)));
      runAction(new actions::ParallelAction(driveAndIntake2));

      runAction(new actions::DriveTrajectory(trajectory::TimingUtil::reverseTimed(path_planning::TrajectorySet::instance->get("backSCurve").get(false))));
      runAction(new actions::OpenLoopDriveAction(util::DriveSignal(-100, -100), 0.5));
  }


    std::list<actions::Action*> driveAndIntakeX;
    driveAndIntakeX.push_back(new actions::OpenLoopIntakeAction(200, 0));
    if(six_cube_)
      driveAndIntakeX.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("backFastForward").get(false)));
    else
      driveAndIntakeX.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("backForward2").get(false)));
    runAction(new actions::ParallelAction(driveAndIntakeX));

    if(six_cube_) {
      std::list<actions::Action*> driveAndIntakeZ;
      driveAndIntakeZ.push_back(new actions::OpenLoopIntakeAction(200, 0));
      driveAndIntakeZ.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("backS2").get(false)));
      runAction(new actions::ParallelAction(driveAndIntakeZ));
    }

    std::list<actions::Action*> driveAndIntake3;
    driveAndIntake3.push_back(new actions::OpenLoopIntakeAction(200, 0));
    driveAndIntake3.push_back(new actions::DriveTrajectory(trajectory::TimingUtil::reverseTimed(path_planning::TrajectorySet::instance->get("backS").get(false))));
    runAction(new actions::ParallelAction(driveAndIntake3));

    //runAction(new actions::OpenLoopDriveAction(util::DriveSignal(flip_ ? -100 : 100, flip_ ? 100: -100), 0.7));

    //runAction(new actions::OpenLoopDriveAction(util::DriveSignal(100.0, 100.0), 1));
    runAction(new actions::DriveTurnAction(-110 * units::degree * (flip_ ? -1 : 1)));

    std::list<actions::Action*> driveAndIntake4;
    driveAndIntake4.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("backSetup").get(flip_)));
    //runAction(new actions::OpenLoopDriveAction(util::DriveSignal(100,100), 0.3));
    driveAndIntake4.push_back(new actions::OpenLoopIntakeAction(200, 0));
    runAction(new actions::ParallelAction(driveAndIntake4));

    runAction(new actions::OpenLoopIntakeAction(-80, 0.75));
    runAction(new actions::TrayEnableStackAction(1.15));
    runAction(new actions::WaitAction(0.3));

    std::list<actions::Action*> pullBackFromStack;
    pullBackFromStack.push_back(new actions::OpenLoopIntakeAction(-100, 0));
    pullBackFromStack.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("stackPullBack").get(flip_)));
    runAction(new actions::ParallelAction(pullBackFromStack));



    /*std::list<actions::Action*> drivesA;
    std::list<actions::Action*> drivesB;
    drivesA.push_back(new actions::WaitAction(0.25));
    drivesA.push_back(new actions::OpenLoopIntakeAction(200, -1));
    drivesB.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("pSkillsIntakeLastTower").get(false)));
    drivesB.push_back(new actions::OpenLoopIntakeAction(200, -1));
    runAction(new actions::ParallelAction(drivesA));
    runAction(new actions::ParallelAction(drivesB));

    std::list<actions::Action*> driveAndIntake2;
    driveAndIntake2.push_back(new actions::DriveTrajectory(trajectory::TimingUtil::reverseTimed(path_planning::TrajectorySet::instance->get("backS").get(flip_))));
    driveAndIntake2.push_back(new actions::OpenLoopIntakeAction(100, 0));
    runAction(new actions::ParallelAction(driveAndIntake2));

    std::list<actions::Action*> turnSetup;
    turnSetup.push_back(new actions::DriveTurnAction(-129 * units::degree * (flip_ ? -1 : 1)));
    turnSetup.push_back(new actions::OpenLoopIntakeAction(200, -1));
    runAction(new actions::ParallelAction(turnSetup));

    runAction(new actions::OpenLoopDriveAction(util::DriveSignal(0.0, 0.0), 0));
    runAction(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("backSetup").get(flip_)));

    runAction(new actions::OpenLoopIntakeAction(-100, 0.8));
    runAction(new actions::TrayEnableStackAction());

    std::list<actions::Action*> pullBackFromStack;
    pullBackFromStack.push_back(new actions::OpenLoopIntakeAction(-200, 1));
    pullBackFromStack.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("stackPullBack").get(flip_)));
    runAction(new actions::ParallelAction(pullBackFromStack)); */

  }

}
