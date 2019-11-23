//
// Created by alexweiss on 8/14/19.
//

#include "AutoModeBase.hpp"
#include "../AutonConstants.hpp"
#include "../actions/DriveTrajectory.hpp"
#include "../actions/WaitAction.hpp"
#include "../actions/OpenLoopDriveAction.hpp"
#include "../actions/OpenLoopIntakeAction.hpp"
#include "../actions/SeriesAction.hpp"
#include "../actions/ParallelAction.hpp"
#include "../actions/TrayEnableStackAction.hpp"
#include "../../paths/TrajectorySet.hpp"
#include "../actions/TrayPosition.hpp"
#include "../actions/OpenLoopLiftAction.hpp"
#include "../actions/ResetLiftTrayPosition.hpp"
#include "main.h"

namespace auton {
  void AutoModeBase::run() {
    active_ = true;
    routine();
    active_ = false;
  }
  void AutoModeBase::stop() {
    active_ = false;
  }
  bool AutoModeBase::isActive() {
    return active_;
  }
  void AutoModeBase::runAction(actions::Action* action) {
    if(!isActive())
      return;
    printf("starting an action\n");
    action->start();
    while(isActive() && !action->isFinished()) {
      action->update();
      pros::Task::delay((int) 1000 / constants::AutonConstants::kAutoUpdateRate);
    }
    action->done();
  }

  void AutoModeBase::flipOut() {
    std::list<actions::Action*> IntakeRelease;
    std::list<actions::Action*> TrayAndOuttakeRelease;

    IntakeRelease.push_back(new actions::WaitAction(0));
    IntakeRelease.push_back(new actions::OpenLoopIntakeAction(-200, 0));

    TrayAndOuttakeRelease.push_back(new actions::SeriesAction(IntakeRelease));
    TrayAndOuttakeRelease.push_back(new actions::TrayPosition(2500, false));
    runAction(new actions::ParallelAction(TrayAndOuttakeRelease));
    runAction(new actions::WaitAction(0.1));

    std::list<actions::Action*> TrayAndLiftDown;
    TrayAndLiftDown.push_back(new actions::OpenLoopLiftAction(50, 0));
    TrayAndLiftDown.push_back(new actions::TrayPosition(500, false));
    runAction(new actions::ParallelAction(TrayAndLiftDown));
    runAction(new actions::ResetLiftTrayPosition());
  }
}
