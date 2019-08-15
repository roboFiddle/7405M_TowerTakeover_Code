//
// Created by alexweiss on 8/14/19.
//

#include "OpenLoopDriveAction.hpp"

namespace auton {
  namespace actions {
    OpenLoopDriveAction::OpenLoopDriveAction(util::DriveSignal signal, units::QTime duration) {
      duration_ = duration;
      left_ = signal.left_voltage();
      right_ = signal.right_voltage();
    }
    bool OpenLoopDriveAction::isFinished() {
      return start_time_ + duration_ < (pros::millis() * units::millisecond);
    }
    void OpenLoopDriveAction::start() {
      printf("started open loop\n");
      start_time_ = pros::millis() * units::millisecond;
      subsystems::Drive::instance->setOpenLoop(util::DriveSignal(left_, right_));
    }
    void OpenLoopDriveAction::update() {

    }
    void OpenLoopDriveAction::done() {
      printf("finished open loop\n");
      subsystems::Drive::instance->setOpenLoop(util::DriveSignal(0.0, 0.0));
    }
  }
}