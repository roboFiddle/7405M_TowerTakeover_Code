//
// Created by alexweiss on 8/14/19.
//

#include "TestMode.hpp"
#include "../actions/DriveTurnAction.hpp"
#include "../actions/LiftPosition.hpp"
#include "../actions/TrayPosition.hpp"
#include "../actions/TrayEnableStackAction.hpp"
#include "../../Constants.hpp"
#include "../../paths/TrajectorySet.hpp"

namespace auton {
  void TestMode::routine() {
    int enabledTest = 3;
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
      runAction(new actions::TrayEnableStackAction());
      runAction(new actions::TrayPosition(1600));
      runAction(new actions::LiftPosition(1800));
    }
  }

}
