#include "TrayPosition.hpp"

namespace auton {
  namespace actions {
    TrayPosition::TrayPosition(double goal) {
      TrayPosition(goal, false);
    }
    TrayPosition::TrayPosition(double goal, bool limit_velo) {
      goal_ = goal;
      limit_velo_ = limit_velo;
    }
    bool TrayPosition::isFinished() {
      return std::fabs(subsystems::Tray::instance->getMotorVelocity()) < 10 && std::fabs(subsystems::Tray::instance->getPositionError()) < 10;
    }
    void TrayPosition::start() {
      subsystems::Tray::instance->setPosition(goal_, limit_velo_);
    }
    void TrayPosition::update() {

    }
    void TrayPosition::done() {

    }
  }
}
