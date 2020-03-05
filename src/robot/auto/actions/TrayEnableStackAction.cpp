#include "TrayEnableStackAction.hpp"

namespace auton {
  namespace actions {
    TrayEnableStackAction::TrayEnableStackAction(double s, int i) {
      s_ = s;
      i_ = i;
    }
    bool TrayEnableStackAction::isFinished() {
        return subsystems::Tray::instance->doneWithScore();
    }
    void TrayEnableStackAction::start() {
        subsystems::Tray::instance->activateScore(s_, i_);
    }
    void TrayEnableStackAction::update() {

    }
    void TrayEnableStackAction::done() {

    }
  }
}
