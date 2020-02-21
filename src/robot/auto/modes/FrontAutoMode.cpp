//
// Created by alexweiss on 10/20/19.
//

#include "FrontAutoMode.hpp"
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
  void FrontAutoMode::routine() {
    flipOut();
    /* std::list<actions::Action*> driveAndIntake;
    driveAndIntake.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("testCode").get(false)));
    runAction(new actions::ParallelAction(driveAndIntake)); */

    std::list<actions::Action*> driveAndIntakeB;
    driveAndIntakeB.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("frontIntake").get(false)));
    driveAndIntakeB.push_back(new actions::OpenLoopIntakeAction(200, -1));
    runAction(new actions::ParallelAction(driveAndIntakeB));

    std::list<actions::Action*> turnSetup;
    turnSetup.push_back(new actions::DriveTurnAction(-45 * units::degree * (flip_ ? -1 : 1)));
    turnSetup.push_back(new actions::OpenLoopIntakeAction(200, -1));
    runAction(new actions::ParallelAction(turnSetup));

    runAction(new actions::OpenLoopDriveAction(util::DriveSignal(0.0, 0.0), 0));

    runAction(new actions::ParallelAction(driveAndIntakeB));

    runAction(new actions::DriveTurnAction(-45 * units::degree * (flip_ ? -1 : 1), false));
    runAction(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("frontSetup").get(false)));
    runAction(new actions::OpenLoopIntakeAction(-50, 0.75));
    std::list<actions::Action*> score;
    score.push_back(new actions::TrayEnableStackAction(110));
    //score.push_back(new actions::OpenLoopIntakeAction(-40, 0));
    runAction(new actions::ParallelAction(score));


    //stacking stack
    std::list<actions::Action*> pullBackFromStack;
    pullBackFromStack.push_back(new actions::OpenLoopIntakeAction(-125, 0));
    pullBackFromStack.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("skillsStackPullBack").get(false)));
    runAction(new actions::ParallelAction(pullBackFromStack));
  }

}
