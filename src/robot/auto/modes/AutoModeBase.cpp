//
// Created by alexweiss on 8/14/19.
//

#include "AutoModeBase.hpp"
#include "../AutonConstants.hpp"
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
      pros::Task::delay(1000 / constants::AutonConstants::kAutoUpdateRate);
    }
    action->done();
  }
}