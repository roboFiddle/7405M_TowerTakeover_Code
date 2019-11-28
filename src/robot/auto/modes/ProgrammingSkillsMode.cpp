#include "ProgrammingSkillsMode.hpp"
#include "../actions/DriveTrajectory.hpp"
#include "../actions/WaitAction.hpp"
#include "../actions/OpenLoopDriveAction.hpp"
#include "../actions/OpenLoopIntakeAction.hpp"
#include "../actions/ParallelAction.hpp"
#include "../actions/TrayEnableStackAction.hpp"
#include "../../paths/TrajectorySet.hpp"
#include "../actions/TrayPosition.hpp"
#include "../actions/OpenLoopLiftAction.hpp"
#include "../actions/DriveTurnAction.hpp"
#include "../actions/ResetLiftTrayPosition.hpp"

namespace auton {
  void ProgrammingSkillsMode::routine() {
    flipOut();

    std::list<actions::Action*> driveAndIntake;
    driveAndIntake.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("programmingSkillsForward").get(false)));
    driveAndIntake.push_back(new actions::OpenLoopIntakeAction(200, -1));
    driveAndIntake.push_back(new actions::OpenLoopLiftAction(50, 0));
    driveAndIntake.push_back(new actions::TrayPosition(500, false));
    runAction(new actions::ParallelAction(driveAndIntake));

    std::list<actions::Action*> turnSetup;
    turnSetup.push_back(new actions::DriveTurnAction(-45 * units::degree));
    turnSetup.push_back(new actions::OpenLoopIntakeAction(200, -1));
    runAction(new actions::ParallelAction(turnSetup));

    runAction(new actions::OpenLoopDriveAction(util::DriveSignal(0.0, 0.0), 0));

    std::list<actions::Action*> bs;
    bs.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("programmingSkillsSetup").get(false)));
    bs.push_back(new actions::OpenLoopIntakeAction(200, -1));
    runAction(new actions::ParallelAction(bs));

    runAction(new actions::OpenLoopIntakeAction(-100, 0.8));
    runAction(new actions::TrayEnableStackAction());

    std::list<actions::Action*> pullBackFromStack;
    pullBackFromStack.push_back(new actions::OpenLoopIntakeAction(-200, 1));
    pullBackFromStack.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("stackPullBack").get(false)));
    runAction(new actions::ParallelAction(pullBackFromStack));

    runAction(new actions::DriveTurnAction(132 * units::degree));
    runAction(new actions::TrayPosition(2000, false));

    std::list<actions::Action*> intakeForTower;
    pullBackFromStack.push_back(new actions::OpenLoopIntakeAction(200, 1));
    pullBackFromStack.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("pSkillsIntakeFirstTower").get(false)));
    runAction(new actions::ParallelAction(pullBackFromStack));

    runAction(new actions::DriveTrajectory(trajectory::TimingUtil::reverseTimed(path_planning::TrajectorySet::instance->get("pSkillsBackFirstTower").get(false))));


  }
}
