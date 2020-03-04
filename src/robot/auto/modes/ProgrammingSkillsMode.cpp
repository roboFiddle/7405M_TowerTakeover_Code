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
#include "../actions/DriveInertialTurnAction.hpp"
#include "../actions/DriveTurnWheelAction.hpp"
#include "../actions/DriveMoveWheelAction.hpp"
#include "../actions/ResetLiftTrayPosition.hpp"
#include "../actions/LiftPosition.hpp"
#include "../../subsystems/Inertial.hpp"
#include "../../Constants.hpp"

namespace auton {
  void ProgrammingSkillsMode::routine() {



        std::list<actions::Action*> liftDown;
        liftDown.push_back(new actions::LiftPosition(300));
        liftDown.push_back(new actions::TrayPosition(500));

        std::list<actions::Action*> deploy;
        deploy.push_back(new actions::OpenLoopIntakeAction(-200, 1));
        deploy.push_back(new actions::OpenLoopDriveAction(util::DriveSignal(-5000, -5000), 2));
        runAction(new actions::ParallelAction(deploy));

        //flipOut();

        runAction(new actions::WaitAction(1.0));

        std::list<actions::Action*> drives;
        drives.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("pSkillsIntake").get(false)));
        drives.push_back(new actions::DriveTrajectory(trajectory::TimingUtil::reverseTimed(path_planning::TrajectorySet::instance->get("backCurve").get(false))));
        drives.push_back(new actions::DriveInertialTurnAction(-40 * units::degree));
        drives.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("pSkillsSetup").get(false)));

        std::list<actions::Action*> driveAndIntake;
        driveAndIntake.push_back(new actions::OpenLoopIntakeAction(200, 0));
        driveAndIntake.push_back(new actions::SeriesAction(drives));
        runAction(new actions::ParallelAction(driveAndIntake));

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
        runAction(new actions::WaitAction(0.5));

        //turning after stacking
        std::list<actions::Action*> C;
        std::list<actions::Action*> D;

        C.push_back(new actions::TrayPosition(1800));
        C.push_back(new actions::LiftPosition(1400));
        D.push_back(new actions::DriveInertialTurnAction(45 * units::degree, false, true));
        D.push_back(new actions::SeriesAction(C));
        runAction(new actions::ParallelAction(D));

        runAction(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("longWallBump").get(false)));
        runAction(new actions::OpenLoopDriveAction(util::DriveSignal(2000, 2000), 1));
        runAction(new actions::WaitAction(0.5));
        subsystems::Inertial::instance->resetRotation();
        runAction(new actions::DriveMoveWheelAction(-12 * units::inch));
        runAction(new actions::WaitAction(0.5));
        runAction(new actions::DriveInertialTurnAction(100 * units::degree, false, true, true));
        runAction(new actions::OpenLoopDriveAction(util::DriveSignal(0, 0), 0));

        runAction(new actions::LiftPosition(300));
        runAction(new actions::TrayPosition(500));


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

        runAction(new actions::DriveMoveWheelAction(-16.5* units::inch));
        runAction(new actions::WaitAction(0.5));
        runAction(new actions::DriveInertialTurnAction(90 * units::degree, false, true));
        runAction(new actions::LiftPosition(300));
        runAction(new actions::TrayPosition(500));

        std::list<actions::Action*> intakeSecondTower;
        intakeSecondTower.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("pSkillsSecondIntake").get(false)));
        intakeSecondTower.push_back(new actions::OpenLoopIntakeAction(200, 0));
        runAction(new actions::ParallelAction(intakeSecondTower));

        runAction(new actions::DriveTrajectory(trajectory::TimingUtil::reverseTimed((path_planning::TrajectorySet::instance->get("longTower").get(false)))));
        runAction(new actions::OpenLoopIntakeAction(-100, 0.5));
        runAction(new actions::TrayPosition(1800));
        runAction(new actions::LiftPosition(2200));
        runAction(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("midTower").get(false)));
        runAction(new actions::OpenLoopIntakeAction(-150, 1));

        std::list<actions::Action*> A;
        std::list<actions::Action*> B;

        A.push_back(new actions::DriveMoveWheelAction(-10 * units::inch));
        A.push_back(new actions::DriveInertialTurnAction(53 * units::degree, false, true));
        B.push_back(new actions::SeriesAction(A));
        B.push_back(new actions::SeriesAction(liftDown));
        runAction(new actions::ParallelAction(B));

        std::list<actions::Action*> getCube;
        std::list<actions::Action*> d;
        d.push_back(new actions::DriveMoveWheelAction(18 * units::inch, false));
        d.push_back(new actions::DriveMoveWheelAction(-6 * units::inch, true));
        getCube.push_back(new actions::OpenLoopIntakeAction(200, 0));
        getCube.push_back(new actions::SeriesAction(d));
        runAction(new actions::ParallelAction(getCube));

        std::list<actions::Action*> turnAndOuttake;
        turnAndOuttake.push_back(new actions::OpenLoopIntakeAction(-100, 0.75));
        turnAndOuttake.push_back(new actions::DriveInertialTurnAction(80 * units::degree, false, true));

        runAction(new actions::ParallelAction(turnAndOuttake));
        runAction(new actions::TrayPosition(1800));
        runAction(new actions::LiftPosition(2200));
        runAction(new actions::DriveMoveWheelAction(8 * units::inch));
        runAction(new actions::OpenLoopIntakeAction(-200, 1));





        /*std::list<actions::Action*> liftTray;
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
        runAction(new actions::OpenLoopDriveAction(util::DriveSignal(0, 0), 0.5)); */





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
