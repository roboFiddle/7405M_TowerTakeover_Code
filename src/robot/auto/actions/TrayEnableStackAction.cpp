#include "TrayEnableStackAction.hpp"

namespace auton {
  namespace actions {
    TrayEnableStackAction::TrayEnableStackAction() {
    }
    bool TrayEnableStackAction::isFinished() {
        return subsystems::Tray::instance->doneWithScore();
    }
    void TrayEnableStackAction::start() {
        subsystems::Tray::instance->activateScore();
    }
    void TrayEnableStackAction::update() {

    }
    void TrayEnableStackAction::done() {

    }
  }
}
