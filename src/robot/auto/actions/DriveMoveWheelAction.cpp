//
// Created by alexweiss on 11/10/19.
//

#include "DriveMoveWheelAction.hpp"
#include "../../subsystems/Drive.hpp"

namespace auton {
  namespace actions {
    DriveMoveWheelAction::DriveMoveWheelAction(units::QLength distance) {
      dist_ = distance;
    }
    bool DriveMoveWheelAction::isFinished() {
      return subsystems::Drive::instance->isDoneWithTrajectory();
    }
    void DriveMoveWheelAction::start() {
      subsystems::Drive::instance->setEncoderWheel(dist_);
    }
    void DriveMoveWheelAction::update() {
    }
    void DriveMoveWheelAction::done() {
      subsystems::Drive::instance->setOpenLoop(util::DriveSignal(0, 0));
    }
  }
}
