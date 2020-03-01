//
// Created by alexweiss on 11/28/19.
//

#include "LiftPosition.hpp"

namespace auton {
  namespace actions {
    LiftPosition::LiftPosition(double goal) {
      goal_ = goal;
      limit_velo_ = false;
    }
    LiftPosition::LiftPosition(double goal, bool limit_velo) {
      goal_ = goal;
      limit_velo_ = limit_velo;
    }
    bool LiftPosition::isFinished() {
      printf("LP %f %f\n", std::fabs(subsystems::Lift::instance->get_demand().getValue()), std::fabs(subsystems::Lift::instance->get_position()));
      return std::fabs(goal_ - subsystems::Lift::instance->get_position()) < 100;
    }
    void LiftPosition::start() {
      printf("LIFT GOAL %f\n", goal_);
      subsystems::Lift::instance->setFromMacro(goal_ );
    }
    void LiftPosition::update() {

    }
    void LiftPosition::done() {
      subsystems::Lift::instance->setOpenLoop(0.0);
    }
  }
}
