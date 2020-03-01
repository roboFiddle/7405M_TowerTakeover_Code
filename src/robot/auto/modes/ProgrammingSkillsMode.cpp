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
#include "../actions/DriveMoveWheelAction.hpp"
#include "../actions/ResetLiftTrayPosition.hpp"
#include "../actions/LiftPosition.hpp"
#include "../../Constants.hpp"

namespace auton {
  void ProgrammingSkillsMode::routine() {
        //flipOut();

        //runAction(new actions::WaitAction(1.0));

        std::list<actions::Action*> drives;
        //drives.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("pSkillsIntake").get(false)));
        /*drives.push_back(new actions::DriveTrajectory(trajectory::TimingUtil::reverseTimed(path_planning::TrajectorySet::instance->get("backCurve").get(false))));
        drives.push_back(new actions::DriveTurnWheelAction(-42 * units::degree));
        drives.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("pSkillsSetup").get(false)));

        std::list<actions::Action*> driveAndIntake;
        driveAndIntake.push_back(new actions::OpenLoopIntakeAction(200, 0));
        driveAndIntake.push_back(new actions::SeriesAction(drives));
        runAction(new actions::ParallelAction(driveAndIntake));*/

        std::list<actions::Action*> score;
        score.push_back(new actions::TrayEnableStackAction(100));
        //score.push_back(new actions::OpenLoopIntakeAction(-40, 0));
        runAction(new actions::ParallelAction(score));

        //intaking cubes for stack
        runAction(new actions::OpenLoopDriveAction(util::DriveSignal(0, 0), 0.5));
        runAction(new actions::OpenLoopIntakeAction(-150, 0.5));

        //stacking stack
        std::list<actions::Action*> pullBackFromStack;
        pullBackFromStack.push_back(new actions::OpenLoopIntakeAction(-125, 0));
        pullBackFromStack.push_back(new actions::DriveMoveWheelAction(-14 * units::inch));
        runAction(new actions::ParallelAction(pullBackFromStack));
        runAction(new actions::WaitAction(1.0));

        //turning after stacking
        runAction(new actions::DriveTurnWheelAction(190 * units::degree));
        runAction(new actions::TrayPosition(300));
        runAction(new actions::OpenLoopDriveAction(util::DriveSignal(-3500, -3500), 1));

        std::list<actions::Action*> intakeFirstTower;
        intakeFirstTower.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("intakeFirstTower").get(false)));
        intakeFirstTower.push_back(new actions::OpenLoopIntakeAction(200, 0));
        runAction(new actions::ParallelAction(intakeFirstTower));
        runAction(new actions::DriveTrajectory(trajectory::TimingUtil::reverseTimed((path_planning::TrajectorySet::instance->get("midTower").get(false)))));
        runAction(new actions::OpenLoopIntakeAction(-100, 0.5));
        runAction(new actions::TrayPosition(1800));
        runAction(new actions::LiftPosition(3300));
        runAction(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("longTower").get(false)));
        runAction(new actions::OpenLoopIntakeAction(-150, 1));

        runAction(new actions::DriveMoveWheelAction(-15.75 * units::inch));
        runAction(new actions::OpenLoopDriveAction(util::DriveSignal(0, 0), 0.5));
        runAction(new actions::DriveTurnWheelAction(110 * units::degree));
        runAction(new actions::LiftPosition(300));
        runAction(new actions::TrayPosition(500));

        std::list<actions::Action*> intakeSecondTower;
        intakeSecondTower.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("pSkillsSecondIntake").get(false)));
        intakeSecondTower.push_back(new actions::OpenLoopIntakeAction(200, 0));
        runAction(new actions::ParallelAction(intakeSecondTower));

        runAction(new actions::DriveTrajectory(trajectory::TimingUtil::reverseTimed((path_planning::TrajectorySet::instance->get("midTower").get(false)))));
        runAction(new actions::OpenLoopIntakeAction(-100, 0.5));
        runAction(new actions::TrayPosition(1800));
        runAction(new actions::LiftPosition(2200));
        runAction(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("midTower").get(false)));
        runAction(new actions::OpenLoopIntakeAction(-150, 1));

        std::list<actions::Action*> liftTray;
        std::list<actions::Action*> align;
        std::list<actions::Action*> fullList;

        liftTray.push_back(new actions::WaitAction(1.0));
        liftTray.push_back(new actions::LiftPosition(300));
        liftTray.push_back(new actions::TrayPosition(500));
        align.push_back(new actions::DriveMoveWheelAction(-20 * units::inch));
        //align.push_back(new actions::DriveTurnWheelAction(-30 * units::degree));

        fullList.push_back(new actions::SeriesAction(liftTray));
        fullList.push_back(new actions::SeriesAction(align));
        runAction(new actions::ParallelAction(fullList));
        runAction(new actions::OpenLoopDriveAction(util::DriveSignal(0, 0), 0.5));





        /*runAction(new actions::OpenLoopDriveAction(util::DriveSignal(0, 0), 0.5));
        runAction(new actions::DriveTrajectory(trajectory::TimingUtil::reverseTimed(path_planning::TrajectorySet::instance->get("alignWithFirstTower").get(false))));
        runAction(new actions::OpenLoopDriveAction(util::DriveSignal(0, 0), 0.5));

                runAction(new actions::OpenLoopDriveAction(util::DriveSignal(0, 0), 1.5));


        //intaking cube for first tower


        runAction(new actions::DriveTrajectory(trajectory::TimingUtil::reverseTimed((path_planning::TrajectorySet::instance->get("longTower").get(false)))));
        runAction(new actions::OpenLoopIntakeAction(-100, 0.5));

        //OLD METHOD FROM LINES 72 - 91
        //getting ready for second tower
        runAction(new actions::TrayPosition(1800));
        runAction(new actions::LiftPosition(3300));
        runAction(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("longTower").get(false)));
        runAction(new actions::OpenLoopIntakeAction(-150, 1));
        //runAction(new actions::DriveTrajectory(trajectory::TimingUtil::reverseTimed((path_planning::TrajectorySet::instance->get("shortTower").get(false)))));

        //curving back to align with second tower
        runAction(new actions::DriveTrajectory(trajectory::TimingUtil::reverseTimed((path_planning::TrajectorySet::instance->get("backCurve").get(false)))));

        runAction(new actions::OpenLoopDriveAction(util::DriveSignal(0, 0), 0.5));
        runAction(new actions::DriveTurnWheelAction(85 * units::degree));
        runAction(new actions::OpenLoopDriveAction(util::DriveSignal(0, 0), 0.5));

        runAction(new actions::LiftPosition(300));
        runAction(new actions::TrayPosition(300));

        //scoring cube for second tower
        std::list<actions::Action*> secondIntake;
        secondIntake.push_back(new actions::OpenLoopIntakeAction(200, 0));
        secondIntake.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("pSkillsSecondIntake").get(false)));
        runAction(new actions::ParallelAction(secondIntake));

        //NEW METHOD
        //turning back to pick up cube for second tower
        runAction(new actions::DriveTrajectory(trajectory::TimingUtil::reverseTimed((path_planning::TrajectorySet::instance->get("longTower").get(false)))));
        runAction(new actions::OpenLoopIntakeAction(-100, 0.5));
        //scoring second tower
        runAction(new actions::TrayPosition(1800));
        runAction(new actions::LiftPosition(2250));
        runAction(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("longTower").get(false)));
        runAction(new actions::OpenLoopIntakeAction(-150, 1)); */










  }
}
