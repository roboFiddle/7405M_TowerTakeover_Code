#include "ResetLiftTrayPosition.hpp"

namespace auton {
  namespace actions {
    ResetLiftTrayPosition::ResetLiftTrayPosition() {
    }
    bool ResetLiftTrayPosition::isFinished() {
      return true;
    }
    void ResetLiftTrayPosition::start() {
      subsystems::Tray::instance->tare();
      subsystems::Lift::instance->tare();
    }
    void ResetLiftTrayPosition::update() {

    }
    void ResetLiftTrayPosition::done() {

    }
  }
}
