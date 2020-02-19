#include "ProgrammingSkillsMode.hpp"
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
#include "../actions/LiftPosition.hpp"
#include "../../Constants.hpp"

namespace auton {
  void ProgrammingSkillsMode::routine() {
        /*std::list<actions::Action*> drives;
        drives.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("pSkillsIntake").get(false)));
        drives.push_back(new actions::DriveTurnWheelAction(-45 * units::degree));
        drives.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("pSkillsSetup").get(false)));

        std::list<actions::Action*> driveAndIntake;
        driveAndIntake.push_back(new actions::OpenLoopIntakeAction(200, 0));
        driveAndIntake.push_back(new actions::SeriesAction(drives));
        runAction(new actions::ParallelAction(driveAndIntake)); */

        std::list<actions::Action*> score;
        score.push_back(new actions::TrayEnableStackAction(1));
        //score.push_back(new actions::OpenLoopIntakeAction(-40, 0));
        runAction(new actions::ParallelAction(score));

        runAction(new actions::WaitAction(0.5));
        runAction(new actions::OpenLoopIntakeAction(-200, 0.67));

        std::list<actions::Action*> pullBackFromStack;
        pullBackFromStack.push_back(new actions::OpenLoopIntakeAction(-125, 0));
        pullBackFromStack.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("skillsStackPullBack").get(false)));
        runAction(new actions::ParallelAction(pullBackFromStack));
        runAction(new actions::WaitAction(1.0));

        runAction(new actions::DriveTurnWheelAction(45 * units::degree));
        runAction(new actions::TrayPosition(1800));
        runAction(new actions::LiftPosition(1400));

        runAction(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("longWallBump").get(false)));
        runAction(new actions::DriveTrajectory(trajectory::TimingUtil::reverseTimed(path_planning::TrajectorySet::instance->get("alignWithFirstTower").get(false))));
        runAction(new actions::WaitAction(0.5));
        runAction(new actions::DriveTurnWheelAction(90 * units::degree));
        runAction(new actions::LiftPosition(300));

        runAction(new actions::TrayPosition(500));

        std::list<actions::Action*> intakeFirstTower;
        intakeFirstTower.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("intakeFirstTower").get(false)));
        intakeFirstTower.push_back(new actions::OpenLoopIntakeAction(200, 0));
        runAction(new actions::ParallelAction(intakeFirstTower));

        runAction(new actions::DriveTrajectory(trajectory::TimingUtil::reverseTimed((path_planning::TrajectorySet::instance->get("shortTower").get(false)))));
        runAction(new actions::OpenLoopIntakeAction(-100, 0.5));
        runAction(new actions::TrayPosition(1800));
        runAction(new actions::LiftPosition(3300));
        runAction(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("longTower").get(false)));
        runAction(new actions::OpenLoopIntakeAction(-150, 1));



  }
}
