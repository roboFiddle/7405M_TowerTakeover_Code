//
// Created by alexweiss on 11/10/19.
//

#include "DriveInertialTurnAction.hpp"
#include "../../subsystems/Drive.hpp"

namespace auton {
  namespace actions {
    DriveInertialTurnAction::DriveInertialTurnAction(units::Angle angle, bool speed, bool r, bool early) {
      angle_ = angle;
      speed_ = speed;
      reset_ = r;
      stop_ = early;
    }
    bool DriveInertialTurnAction::isFinished() {
      return subsystems::Drive::instance->isDoneWithTrajectory();
    }
    void DriveInertialTurnAction::start() {
      subsystems::Drive::instance->setInertialTurn(angle_, speed_, reset_);
    }
    void DriveInertialTurnAction::update() {
    }
    void DriveInertialTurnAction::done() {
      subsystems::Drive::instance->setOpenLoop(util::DriveSignal(0, 0));
    }
  }
}
