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
    last_error = 0;
  }
  units::Number Lift::get_demand() {
    return demand;
  }
  double Lift::getPositionError() {
    return demand.getValue() - pot->get_value();
  }
  double Lift::getMotorVelocity() {
    return motor->get_actual_velocity();
  }
  double Lift::get_position() {
    return motor->get_position();
  }
  ControlState Lift::getState() {
    return state;
  }
  double Lift::getTrayForDemand() {
    if(demand.getValue() > pot->get_value()) { // going up
      if(demand.getValue() < constants::RobotConstants::LIFT_STAGE[1]) { // levels 1/2
        return constants::RobotConstants::TRAY_LIFT[1];
      }
      else { // level 3
        return constants::RobotConstants::TRAY_LIFT[3];
      }
    }
    else { // going down
      if(demand.getValue() < constants::RobotConstants::LIFT_STAGE[2]) { // level 0
        if(pot->get_value() > constants::RobotConstants::LIFT_STAGE[1]) { // top of two level drop
          return constants::RobotConstants::TRAY_LIFT[2];
        }
        else {
          return constants::RobotConstants::TRAY_LIFT[1];
        }
      }
      else { // going down to 1/2
        return constants::RobotConstants::TRAY_LIFT[1];
      }
    }
  }
  void Lift::runPID() {
    double error = demand.getValue() - pot->get_value();
    double error_change = error - last_error;
    motor->move_velocity( (int) (error * -0.5 + error_change * -10));
    last_error = error;
  }
  void Lift::updateOutputs() {
    //("LIFT MOTOR STATE %f %d %d\n", motor->get_temperature(), motor->is_over_current(), motor->is_over_temp());
    if(state == ControlState::OPEN_LOOP)
      motor->move_velocity(demand.getValue());
    else if(state == ControlState::POSITION_CONTROL) {
      Tray::instance->setPosition(getTrayForDemand(), false);
      lastTray = getTrayForDemand();
      double tray_error = std::fabs(Tray::instance->get_position() - getTrayForDemand());
      //printf("LIFT MACRO %f %f\n", lastTray, tray_error);
      if(tray_error < 100 || Tray::instance->get_position() > 700 && lastTray < 1900)
        runPID();
    }
  }
  void Lift::stop() {
    setOpenLoop(0);
  }
  void Lift::tare() {
    printf("TARE %f \n", pros::c::motor_get_position(constants::RobotConstants::motor_lift));
    pros::c::motor_tare_position(constants::RobotConstants::motor_lift);
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
