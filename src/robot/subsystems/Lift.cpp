#include "Lift.hpp"
#include "Tray.hpp"
#include "../Constants.hpp"

#include <stdio.h>

namespace subsystems {
  Lift::LiftManager Lift::instance;

  Lift::Lift() {
    motor = new pros::Motor(constants::RobotConstants::motor_lift, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    motor->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    state = ControlState::OPEN_LOOP;
  }

  void Lift::setOpenLoop(units::Number control) {
    state = ControlState::OPEN_LOOP;
    demand = control;
  }
  void Lift::setPosition(units::Number control) {
    state = ControlState::POSITION_CONTROL;
    demand = control;
  }
  units::Number Lift::get_demand() {
    return demand;
  }
  double Lift::get_position() {
    return motor->get_position();
  }
  ControlState Lift::getState() {
    return state;
  }
  double Lift::getTrayForDemand() {
    if(demand.getValue() < constants::RobotConstants::LIFT_STAGE[0]) {
      return constants::RobotConstants::TRAY_LIFT[0];
    }
    else if(demand.getValue() < constants::RobotConstants::LIFT_STAGE[1]) {
      return constants::RobotConstants::TRAY_LIFT[1];
    }
    else {
      return constants::RobotConstants::TRAY_LIFT[2];
    }
  }
  void Lift::updateOutputs() {
    if(state == ControlState::OPEN_LOOP)
      motor->move_velocity(demand.getValue());
    else if(state == ControlState::POSITION_CONTROL) {
      Tray::instance->setPosition(getTrayForDemand());
      if(Tray::instance->get_position() > getTrayForDemand()*.85)
        motor->move_absolute(demand.getValue(), constants::RobotConstants::MAX_LIFT_RPM);
    }
  }
  void Lift::stop() {
    setOpenLoop(0);
  }
  void Lift::registerEnabledLoops(loops::Looper* enabledLooper) {
    loops::Loop* thisLoop = new loops::Loop();
    thisLoop->onStart = []() {};
    thisLoop->onLoop = []() {
      Lift::instance->updateOutputs();
    };
    thisLoop->onDisable = []() {};
    enabledLooper->add(std::shared_ptr<loops::Loop>(thisLoop));
  }

}
