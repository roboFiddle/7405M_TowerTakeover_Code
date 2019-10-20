//
// Created by alexweiss on 8/14/19.
//

#include "TestMode.hpp"
#include "../../Constants.hpp"
#include "../../paths/TrajectorySet.hpp"

namespace auton {
  void TestMode::routine() {
    printf("started test mode\n");
    runAction(new actions::OpenLoopDriveAction(util::DriveSignal(100.0, 100.0), 1.0));
    runAction(new actions::WaitAction(1.5));
    runAction(new actions::OpenLoopDriveAction(util::DriveSignal(-100.0, -100.0), 1.0));
  }

}
