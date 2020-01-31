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
#include "../../subsystems/Odometry.hpp"

namespace auton {
  void BackAutoMode::routine() {

    flipOut();

    runAction(new actions::OpenLoopTrayAction(100, 1));
    runAction(new actions::OpenLoopLiftAction(-150, 0.5));
    runAction(new actions::OpenLoopLiftAction(150, 0.5));
    runAction(new actions::OpenLoopTrayAction(-100, 1));

    std::list<actions::Action*> driveAndIntake;
    driveAndIntake.push_back(new actions::OpenLoopIntakeAction(200, 0));
    driveAndIntake.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("backJForward").get(false)));
    runAction(new actions::ParallelAction(driveAndIntake));

    std::list<actions::Action*> driveAndIntake2;
    driveAndIntake2.push_back(new actions::OpenLoopIntakeAction(200, 0));
    driveAndIntake2.push_back(new actions::DriveTurnAction(30 * units::degree * (flip_ ? -1 : 1)));
    runAction(new actions::ParallelAction(driveAndIntake2));

    std::list<actions::Action*> driveAndIntakeR;
    driveAndIntakeR.push_back(new actions::OpenLoopIntakeAction(200, 0));
    driveAndIntakeR.push_back(new actions::DriveTrajectory(trajectory::TimingUtil::reverseTimed(path_planning::TrajectorySet::instance->get("backToLine").get(false))));
    runAction(new actions::ParallelAction(driveAndIntakeR));

    runAction(new actions::OpenLoopDriveAction(util::DriveSignal(-200, -200), 0.5));

    std::list<actions::Action*> driveAndIntakeX;
    driveAndIntakeX.push_back(new actions::OpenLoopIntakeAction(200, 0));
    driveAndIntakeX.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("backLineForward").get(false)));
    runAction(new actions::ParallelAction(driveAndIntakeX));

    std::list<actions::Action*> driveAndIntakeA;
    driveAndIntakeA.push_back(new actions::OpenLoopIntakeAction(200, 0));
    driveAndIntakeA.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("backCube").get(false)));
    runAction(new actions::ParallelAction(driveAndIntakeA));
    if(flip_) {
      std::list<actions::Action*> driveAndIntakeF;
      driveAndIntakeF.push_back(new actions::OpenLoopIntakeAction(200, 0));
      driveAndIntakeF.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("backCubeShort").get(false)));
      runAction(new actions::ParallelAction(driveAndIntakeF));
    }
    else {
      runAction(new actions::ParallelAction(driveAndIntakeA));
    }

    //runAction(new actions::DriveTurnWheelAction(-1 * subsystems::Odometry::instance->getPosition().rotation().getRadians()));


    /* std::list<actions::Action*> driveAndIntake3;
    driveAndIntake3.push_back(new actions::OpenLoopIntakeAction(200, 0));
    if(flip_)
      driveAndIntake3.push_back(new actions::DriveTrajectory(trajectory::TimingUtil::reverseTimed(path_planning::TrajectorySet::instance->get("backAlignShort").get(false))));
    else
      driveAndIntake3.push_back(new actions::DriveTrajectory(trajectory::TimingUtil::reverseTimed(path_planning::TrajectorySet::instance->get("backAlign").get(false))));
    runAction(new actions::ParallelAction(driveAndIntake3)); */

    /*runAction(new actions::DriveTurnWheelAction(-133 * units::degree * (flip_ ? -1 : 1) - subsystems::Odometry::instance->getPosition().rotation().getRadians()));

    std::list<actions::Action*> driveAndIntake4;
    //driveAndIntake4.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("backSetup").get(flip_)));
    driveAndIntake4.push_back(new actions::OpenLoopDriveAction(util::DriveSignal(200, 200), .5));
    driveAndIntake4.push_back(new actions::OpenLoopIntakeAction(200, 0));
    runAction(new actions::ParallelAction(driveAndIntake4)); */

    /* runAction(new actions::WaitAction(0.5));
    runAction(new actions::OpenLoopIntakeAction(-65, 0.6));

    std::list<actions::Action*> score;
    score.push_back(new actions::TrayEnableStackAction(0.85));
    //score.push_back(new actions::OpenLoopDriveAction(util::DriveSignal(70, 70), 0.5));
    runAction(new actions::ParallelAction(score));
    runAction(new actions::WaitAction(0.3));

    std::list<actions::Action*> pullBackFromStack;
    pullBackFromStack.push_back(new actions::OpenLoopIntakeAction(-125, 0));
    pullBackFromStack.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("stackPullBack").get(flip_)));
    runAction(new actions::ParallelAction(pullBackFromStack));

    runAction(new actions::OpenLoopTrayAction(-100, 0.7));
    runAction(new actions::OpenLoopLiftAction(-150, 0.5));
    runAction(new actions::OpenLoopLiftAction(150, 0.5)); */

  }

}
