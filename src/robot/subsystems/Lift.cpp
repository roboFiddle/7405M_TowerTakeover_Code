#include "Lift.hpp"
#include "Tray.hpp"
#include "../Constants.hpp"

#include <stdio.h>

namespace subsystems {
  Lift::LiftManager Lift::instance;

  Lift::Lift() {
    motor = new pros::Motor(constants::RobotConstants::motor_lift, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    pot = new pros::ADIAnalogIn(2);
    motor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    state = ControlState::OPEN_LOOP;
    lastTray = 0;
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
    if(demand.getValue() > constants::RobotConstants::LIFT_STAGE[0]) {
      if(pot->get_value() > 2000)
        return constants::RobotConstants::TRAY_LIFT[2];
      else if(pot->get_value() > 1900)
        return constants::RobotConstants::TRAY_LIFT[1];
      else
        return constants::RobotConstants::TRAY_LIFT[0];
    }
    else if(demand.getValue() > constants::RobotConstants::LIFT_STAGE[1]) {
      return constants::RobotConstants::TRAY_LIFT[2];
    }
    else {
      if(pot->get_value() > 2000)
        return constants::RobotConstants::TRAY_LIFT[3];
      else
        return constants::RobotConstants::TRAY_LIFT[2];
    }
  }
  void Lift::updateOutputs() {
    if(state == ControlState::OPEN_LOOP)
      motor->move_velocity(demand.getValue());
    else if(state == ControlState::POSITION_CONTROL) {
      Tray::instance->setPosition(getTrayForDemand());
      lastTray = getTrayForDemand();
      double e = std::fabs(Tray::instance->get_position() - getTrayForDemand());
      if(e < std::fabs(getTrayForDemand()*0.8))
        motor->move_absolute(demand.getValue(), 200);
    }
  }
  void Lift::stop() {
    setOpenLoop(0);
  }
  void Lift::tare() {
    motor->tare_position();
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
