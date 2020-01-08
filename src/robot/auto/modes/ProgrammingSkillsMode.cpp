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
#include "../actions/ResetLiftTrayPosition.hpp"
#include "../actions/LiftPosition.hpp"
#include "../../Constants.hpp"

namespace auton {
  void ProgrammingSkillsMode::routine() {
    std::list<actions::Action*> IntakeRelease;
    std::list<actions::Action*> TrayAndOuttakeRelease;
    std::list<actions::Action*> drives;

    IntakeRelease.push_back(new actions::WaitAction(0.25));
    IntakeRelease.push_back(new actions::OpenLoopIntakeAction(150, 0.25));
    IntakeRelease.push_back(new actions::WaitAction(0.1));
    IntakeRelease.push_back(new actions::OpenLoopIntakeAction(-150, 0));

    drives.push_back(new actions::OpenLoopDriveAction(util::DriveSignal(150, 150), 0.35));
    drives.push_back(new actions::OpenLoopDriveAction(util::DriveSignal(-50, -50), 1));



    TrayAndOuttakeRelease.push_back(new actions::SeriesAction(IntakeRelease));
    TrayAndOuttakeRelease.push_back(new actions::SeriesAction(drives));
    TrayAndOuttakeRelease.push_back(new actions::TrayPosition(2750, false));
    runAction(new actions::ParallelAction(TrayAndOuttakeRelease));
    runAction(new actions::WaitAction(0.1));

    std::list<actions::Action*> TrayAndLiftDown;
    TrayAndLiftDown.push_back(new actions::OpenLoopLiftAction(50, 0));
    TrayAndLiftDown.push_back(new actions::OpenLoopTrayAction(-200, 0.5));
    //TrayAndLiftDown.push_back(new actions::TrayPosition(1800, false));
    runAction(new actions::ParallelAction(TrayAndLiftDown));
    runAction(new actions::OpenLoopLiftAction(0, 0));
    printf("TIME %d", pros::millis());

    //Pick Up Line of Cubes
    std::list<actions::Action*> driveAndIntake;
    driveAndIntake.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("programmingSkillsForward").get(false)));
    driveAndIntake.push_back(new actions::OpenLoopIntakeAction(200, -1));
    driveAndIntake.push_back(new actions::OpenLoopLiftAction(50, 0));
    driveAndIntake.push_back(new actions::TrayPosition(500, false));
    runAction(new actions::ParallelAction(driveAndIntake));
    runAction(new actions::OpenLoopLiftAction(0, 0));
    runAction(new actions::DriveTrajectory(trajectory::TimingUtil::reverseTimed(path_planning::TrajectorySet::instance->get("pSkillsBackTower").get(false))));

    // Turn to Line Up With Scoring Zone
    std::list<actions::Action*> turnSetup;
    turnSetup.push_back(new actions::DriveTurnAction(-45 * units::degree));
    turnSetup.push_back(new actions::OpenLoopIntakeAction(200, -1));
    runAction(new actions::ParallelAction(turnSetup));
    runAction(new actions::OpenLoopDriveAction(util::DriveSignal(0.0, 0.0), 0));

    // Move Into Scoring Zone
    std::list<actions::Action*> bs;
    bs.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("programmingSkillsSetup").get(false)));
    bs.push_back(new actions::OpenLoopIntakeAction(200, 1.0));
    runAction(new actions::ParallelAction(bs));
    runAction(new actions::OpenLoopIntakeAction(200, 1.0));


    // Place Stack in Scoring Zone
    runAction(new actions::OpenLoopIntakeAction(-100, 0.8));
    runAction(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("pSkillsBackTower").get(false)));
    runAction(new actions::TrayEnableStackAction(2));

    runAction(new actions::WaitAction(1));

    // Pull Back From Stack
    std::list<actions::Action*> pullBackFromStack;
    std::list<actions::Action*> pullBackFromStackDrive;
    pullBackFromStackDrive.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("stackPullBack").get(false)));

    pullBackFromStack.push_back(new actions::OpenLoopIntakeAction(-200, 1));
    pullBackFromStack.push_back(new actions::SeriesAction(pullBackFromStackDrive));
    runAction(new actions::ParallelAction(pullBackFromStack));

    // Align With First Tower
    runAction(new actions::DriveTurnAction(135 * units::degree));

    std::list<actions::Action*> alignWall;
    alignWall.push_back(new actions::OpenLoopDriveAction(util::DriveSignal(-100, -100), 1));
    alignWall.push_back(new actions::TrayPosition(1750, false));
    runAction(new actions::ParallelAction(pullBackFromStack));

    // Intake Cube for First Tower
    std::list<actions::Action*> intakeForTowerDrive;
    intakeForTowerDrive.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("pSkillsIntakeFirstTower").get(false)));
    intakeForTowerDrive.push_back(new actions::WaitAction(0.5));
    intakeForTowerDrive.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("pSkillsIntakeCloseTower").get(false)));
    std::list<actions::Action*> intakeForTower;
    intakeForTower.push_back(new actions::OpenLoopIntakeAction(200, 0));
    intakeForTower.push_back(new actions::SeriesAction(intakeForTowerDrive));
    runAction(new actions::ParallelAction(intakeForTower));

    // Place Cube into First Tower
    runAction(new actions::DriveTrajectory(trajectory::TimingUtil::reverseTimed(path_planning::TrajectorySet::instance->get("pSkillsBackTower").get(false))));
    runAction(new actions::LiftPosition(constants::RobotConstants::LIFT_PRESETS[2]));
    runAction(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("pSkillsBackTower").get(false)));
    runAction(new actions::OpenLoopIntakeAction(-200, 1));
    runAction(new actions::DriveTrajectory(trajectory::TimingUtil::reverseTimed(path_planning::TrajectorySet::instance->get("pSkillsBackTower").get(false))));
    runAction(new actions::LiftPosition(constants::RobotConstants::LIFT_PRESETS[0]));
    runAction(new actions::OpenLoopLiftAction(0, 0));

    // Line Up with Second Tower
    runAction(new actions::DriveTrajectory(trajectory::TimingUtil::reverseTimed(path_planning::TrajectorySet::instance->get("pSkillsLongBackSecondTower").get(false))));
    runAction(new actions::DriveTurnAction(90 * units::degree));

    // Intake for Second Tower
    std::list<actions::Action*> intakeForTowerDrive2;
    intakeForTowerDrive2.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("pSkillsIntakeSecondTower").get(false)));
    intakeForTowerDrive2.push_back(new actions::WaitAction(0.5));
    std::list<actions::Action*> intakeForTower2;
    intakeForTower2.push_back(new actions::OpenLoopIntakeAction(200, 0));
    intakeForTower2.push_back(new actions::SeriesAction(intakeForTowerDrive));
    runAction(new actions::ParallelAction(intakeForTower2));

    // Place into Second Tower
    runAction(new actions::DriveTrajectory(trajectory::TimingUtil::reverseTimed(path_planning::TrajectorySet::instance->get("pSkillsBackTower").get(false))));
    runAction(new actions::LiftPosition(constants::RobotConstants::LIFT_PRESETS[1]));
    runAction(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("pSkillsBackTower").get(false)));
    runAction(new actions::OpenLoopIntakeAction(-200, 1));
    runAction(new actions::DriveTrajectory(trajectory::TimingUtil::reverseTimed(path_planning::TrajectorySet::instance->get("pSkillsBackTower").get(false))));
    runAction(new actions::LiftPosition(constants::RobotConstants::LIFT_PRESETS[0]));
    runAction(new actions::OpenLoopLiftAction(0, 0));

    // Line Up for Second Stack
    runAction(new actions::DriveTrajectory(trajectory::TimingUtil::reverseTimed(path_planning::TrajectorySet::instance->get("pSkillsIntakeSecondTower").get(false))));
    runAction(new actions::DriveTurnAction(-90 * units::degree));
    runAction(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("pSkillsLineUpSecondStack").get(false)));
    runAction(new actions::DriveTurnAction(90 * units::degree));
    runAction(new actions::TrayPosition(500, false));

    // Intake For Second Stack
    std::list<actions::Action*> driveAndIntake2;
    driveAndIntake2.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("programmingSkillsForward").get(false)));
    driveAndIntake2.push_back(new actions::OpenLoopIntakeAction(200, -1));
    runAction(new actions::ParallelAction(driveAndIntake2));

    // Line Up with Second Scoring Zone
    runAction(new actions::DriveTrajectory(trajectory::TimingUtil::reverseTimed(path_planning::TrajectorySet::instance->get("pSkillsLineUpBackSecondStack").get(false))));
    runAction(new actions::DriveTurnAction(90 * units::degree));
    runAction(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("pSkillsLineUpForwardSecondStack").get(false)));

    // Score Second Stack
    runAction(new actions::ParallelAction(turnSetup));
    runAction(new actions::OpenLoopDriveAction(util::DriveSignal(0.0, 0.0), 0));
    runAction(new actions::ParallelAction(bs));
    runAction(new actions::OpenLoopIntakeAction(-100, 0.8));
    runAction(new actions::TrayEnableStackAction());
    runAction(new actions::ParallelAction(pullBackFromStack));

    // Line up With Third Tower
    runAction(new actions::DriveTurnAction(-132 * units::degree));
    runAction(new actions::TrayPosition(1400, false));
    runAction(new actions::ParallelAction(intakeForTower));

    // Score in Third Tower
    runAction(new actions::DriveTrajectory(trajectory::TimingUtil::reverseTimed(path_planning::TrajectorySet::instance->get("pSkillsBackTower").get(false))));
    runAction(new actions::LiftPosition(constants::RobotConstants::LIFT_PRESETS[2]));
    runAction(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("pSkillsBackTower").get(false)));
    runAction(new actions::OpenLoopIntakeAction(-200, 1));
    runAction(new actions::DriveTrajectory(trajectory::TimingUtil::reverseTimed(path_planning::TrajectorySet::instance->get("pSkillsBackTower").get(false))));
    runAction(new actions::LiftPosition(constants::RobotConstants::LIFT_PRESETS[0]));
    runAction(new actions::OpenLoopLiftAction(0, 0));

    // Fourth Tower
    runAction(new actions::DriveTurnAction(-45 * units::degree));
    std::list<actions::Action*> intakeForTowerDriveLastTower;
    intakeForTowerDriveLastTower.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("pSkillsIntakeLastTower").get(false)));
    intakeForTowerDriveLastTower.push_back(new actions::WaitAction(0.5));
    std::list<actions::Action*> intakeForLastTower;
    intakeForLastTower.push_back(new actions::OpenLoopIntakeAction(200, 0));
    intakeForLastTower.push_back(new actions::SeriesAction(intakeForTowerDriveLastTower));
    runAction(new actions::ParallelAction(intakeForLastTower));
    runAction(new actions::DriveTrajectory(trajectory::TimingUtil::reverseTimed(path_planning::TrajectorySet::instance->get("pSkillsBackTower").get(false))));
    runAction(new actions::LiftPosition(constants::RobotConstants::LIFT_PRESETS[1]));
    runAction(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("pSkillsBackTower").get(false)));
    runAction(new actions::OpenLoopIntakeAction(-200, 1));
    runAction(new actions::DriveTrajectory(trajectory::TimingUtil::reverseTimed(path_planning::TrajectorySet::instance->get("pSkillsBackTower").get(false))));






  }
}
