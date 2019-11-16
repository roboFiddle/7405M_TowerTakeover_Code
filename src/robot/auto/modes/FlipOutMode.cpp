//
// Created by alexweiss on 8/14/19.
//

#include "FlipOutMode.hpp"
#include "../actions/DriveTrajectory.hpp"
#include "../actions/WaitAction.hpp"
#include "../actions/OpenLoopDriveAction.hpp"
#include "../actions/OpenLoopIntakeAction.hpp"
#include "../actions/ParallelAction.hpp"
#include "../actions/TrayEnableStackAction.hpp"
#include "../../paths/TrajectorySet.hpp"
#include "../actions/TrayPosition.hpp"
#include "../actions/OpenLoopLiftAction.hpp"
#include "../actions/ResetLiftTrayPosition.hpp"

namespace auton {
  void FlipOutMode::routine() {
    runAction(new actions::OpenLoopDriveAction(util::DriveSignal(200, 200), 0.3));
    runAction(new actions::OpenLoopDriveAction(util::DriveSignal(-200, -200), 0.5));
    runAction(new actions::OpenLoopIntakeAction(200, 0.6));
    std::list<actions::Action*> TrayAndOuttakeRelease;
    TrayAndOuttakeRelease.push_back(new actions::TrayPosition(1000, false));
    TrayAndOuttakeRelease.push_back(new actions::OpenLoopIntakeAction(-200, 0));
    runAction(new actions::ParallelAction(TrayAndOuttakeRelease));

    std::list<actions::Action*> TrayAndLiftDown;
    TrayAndLiftDown.push_back(new actions::OpenLoopLiftAction(50, 0));
    TrayAndLiftDown.push_back(new actions::TrayPosition(0, false));
    runAction(new actions::ParallelAction(TrayAndLiftDown));
    runAction(new actions::ResetLiftTrayPosition());
  }

}
