//
// Created by alexweiss on 8/14/19.
//

#include "TestMode.hpp"
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
#include "../actions/DriveInertialTurnAction.hpp"
#include "../actions/DriveTurnAction.hpp"
#include "../actions/DriveTurnWheelAction.hpp"
#include "../actions/ResetLiftTrayPosition.hpp"
#include "../actions/LiftPosition.hpp"
#include "../../subsystems/Odometry.hpp"
#include "../../Constants.hpp"

namespace auton {
  void TestMode::routine() {
    int enabledTest = 6;
    // TEST 0 - Lift Position
    if(enabledTest == 0) {
      runAction(new actions::TrayPosition(1750, false));
      runAction(new actions::LiftPosition(1500.0));
      runAction(new actions::LiftPosition(1200.0));
      runAction(new actions::LiftPosition(1800.0));
      runAction(new actions::LiftPosition(1200.0));
      runAction(new actions::LiftPosition(2000.0));
    }
    // TEST 1 - Tray Position
    if(enabledTest == 1) {
      runAction(new actions::TrayPosition(1750));
      runAction(new actions::TrayPosition(400));
      runAction(new actions::TrayPosition(1600));
      runAction(new actions::TrayPosition(400));
      runAction(new actions::TrayPosition(2000));
    }
    // TEST 2 - Lift & Tray Position
    if(enabledTest == 2) {
      runAction(new actions::TrayPosition(1750));
      runAction(new actions::LiftPosition(1800));
    }
    // TEST 3 - Stack & Tray
    if(enabledTest == 3) {
      runAction(new actions::TrayEnableStackAction());
      runAction(new actions::TrayPosition(1600));
    }
    // TEST 4 - Stack & Tray/Lift
    if(enabledTest == 4) {
      runAction(new actions::TrayPosition(1800));
      runAction(new actions::LiftPosition(3300));
      runAction(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("longTower").get(false)));
      runAction(new actions::OpenLoopIntakeAction(-150, 1));

      runAction(new actions::DriveTrajectory(trajectory::TimingUtil::reverseTimed((path_planning::TrajectorySet::instance->get("backCurve").get(true)))));
      runAction(new actions::LiftPosition(300));
      runAction(new actions::TrayPosition(300));
      //runAction(new actions::DriveTurnWheelAction(10 * units::degree));
    }
    if(enabledTest == 5) {
      runAction(new actions::DriveTurnWheelAction(90 * units::degree));
      pros::lcd::print(4, "%f", subsystems::Odometry::instance->getPosition().rotation().getDegrees());
      pros::Task::delay(500);
      runAction(new actions::DriveTurnWheelAction(180 * units::degree));
      pros::lcd::print(5, "%f", subsystems::Odometry::instance->getPosition().rotation().getDegrees());
      pros::Task::delay(500);
      runAction(new actions::DriveTurnWheelAction(-90 * units::degree));
      pros::lcd::print(6, "%f", subsystems::Odometry::instance->getPosition().rotation().getDegrees());
      pros::Task::delay(500);
      runAction(new actions::DriveTurnWheelAction(-180 * units::degree));
      pros::lcd::print(7, "%f", subsystems::Odometry::instance->getPosition().rotation().getDegrees());
      //pros::Task::delay(500);
    }
    if(enabledTest == 6) {
      runAction(new actions::DriveInertialTurnAction(90 * units::degree, false, false));
      //runAction(new actions::DriveInertialTurnAction(-90 * units::degree));
      //runAction(new actions::DriveInertialTurnAction(180 * units::degree));
      //runAction(new actions::DriveInertialTurnAction(-180 * units::degree));
    }
  }

}
