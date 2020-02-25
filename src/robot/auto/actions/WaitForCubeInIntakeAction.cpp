#include "WaitForCubeInIntakeAction.hpp"

namespace auton {
  namespace actions {
    WaitForCubeInIntakeAction::WaitForCubeInIntakeAction() {
      activated = false;
    }
    bool WaitForCubeInIntakeAction::isFinished() {
      return activated;
    }
    void WaitForCubeInIntakeAction::start() {

    }
    void WaitForCubeInIntakeAction::update() {
      activated = subsystems::VisionSystem::instance->cubeInIntake();
    }
    void WaitForCubeInIntakeAction::done() {

    }
  }
}
