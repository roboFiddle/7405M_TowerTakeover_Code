//
// Created by alexweiss on 11/28/19.
//

#include "LiftPosition.hpp"

namespace auton {
  namespace actions {
    LiftPosition::LiftPosition(double goal) {
      LiftPosition(goal, false);
    }
    LiftPosition::LiftPosition(double goal, bool limit_velo) {
      goal_ = goal;
      limit_velo_ = limit_velo;
    }
    bool LiftPosition::isFinished() {
      //printf("TP %f %f\n", std::fabs(subsystems::Tray::instance->getMotorVelocity()), std::fabs(subsystems::Tray::instance->getPositionError()));
      return std::fabs(subsystems::Lift::instance->getMotorVelocity()) < 15 && std::fabs(subsystems::Lift::instance->getPositionError()) < 100;
    }
    void LiftPosition::start() {
      subsystems::Lift::instance->setPosition(goal_);
    }
    void LiftPosition::update() {

    }
    void LiftPosition::done() {
      subsystems::Lift::instance->setOpenLoop(0.0);
    }
  }
}
