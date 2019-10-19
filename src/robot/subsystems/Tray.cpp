#include "Tray.hpp"
#include "../Constants.hpp"

#include <stdio.h>

namespace subsystems {
  Tray::TrayManager Tray::instance;

  Tray::Tray() {
    motor = new pros::Motor(constants::RobotConstants::motor_tray, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_DEGREES);
    motor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    current_state = ControlState::OPEN_LOOP;
  }

  void Tray::setOpenLoop(units::Number control) {
    current_state = ControlState::OPEN_LOOP;
    demand = control;
  }
  void Tray::setPosition(units::Number control) {
    current_state = ControlState::POSITION_CONTROL;
    demand = control;
  }
  double Tray::get_position() {
    return motor->get_position();
  }
  void Tray::updateOutputs() {
    if(current_state == ControlState::OPEN_LOOP)
      motor->move_velocity(demand.getValue());
    else if(current_state == ControlState::POSITION_CONTROL)
      motor->move_absolute(demand.getValue(), constants::RobotConstants::MAX_TRAY_RPM);
  }
  ControlState Tray::getState() {
    return current_state;
  }
  void Tray::stop() {
    setOpenLoop(0);
  }
  void Tray::registerEnabledLoops(loops::Looper* enabledLooper) {
    loops::Loop* thisLoop = new loops::Loop();
    thisLoop->onStart = []() {};
    thisLoop->onLoop = []() {
      Tray::instance->updateOutputs();
    };
    thisLoop->onDisable = []() {};
    enabledLooper->add(std::shared_ptr<loops::Loop>(thisLoop));
  }
  Tray::~Tray() {
    delete motor;
  }
}
