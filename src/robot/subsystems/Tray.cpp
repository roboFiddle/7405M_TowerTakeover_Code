#include "Tray.hpp"
#include "Drive.hpp"
#include "Intake.hpp"
#include "../Constants.hpp"

#include <stdio.h>

namespace subsystems {
  Tray::TrayManager Tray::instance;

  Tray::Tray() {
    motor = new pros::Motor(constants::RobotConstants::motor_tray, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_DEGREES);
    pot = new pros::ADIAnalogIn(1);
    motor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    current_state = ControlState::OPEN_LOOP;
    limit_velo_ = true;
  }

  void Tray::setOpenLoop(units::Number control) {
    current_state = ControlState::OPEN_LOOP;
    demand = control;
    limit_velo_ = true;
  }
  void Tray::setPosition(units::Number control) {
    setPosition(control, true);
  }
  void Tray::setPosition(units::Number control, bool limit_velo) {
    current_state = ControlState::POSITION_CONTROL;
    demand = control;
    limit_velo_ = limit_velo;
  }
  double Tray::getMultiplier() {
    if(!limit_velo_)
      return 1.5;

    if(pot->get_value() < 1600) {
      return 1;
    }
    else if(pot->get_value() > 2750) {
      return -.05;
    }
    else {
      return 1 - (pot->get_value() - 1400) * (0.5/1550);
    }
  }
  double Tray::get_position() {
    return pot->get_value();
  }
  void Tray::runPID() {
    double error = demand.getValue() - pot->get_value();
    double m = limit_velo_ ? 0.05 : .2;
    motor->move_velocity((int) error * m);
  }
  void Tray::updateOutputs() {
    if(current_state == ControlState::OPEN_LOOP)
      motor->move_velocity(demand.getValue() * (demand.getValue() > 0 ? getMultiplier() : 1));
    else if(current_state == ControlState::POSITION_CONTROL)
      runPID();
    else if(current_state == ControlState::SCORE_TRAY) {
      demand = constants::RobotConstants::TRAY_SCORE;
      runPID();
      if(std::fabs(pot->get_value() - constants::RobotConstants::SCORE_START_INTAKE) < 200 || std::fabs(pot->get_value() - constants::RobotConstants::SCORE_END_INTAKE) < 200) {
        Intake::instance->setFromMacro(200);
        //Drive::instance->setFromMacro(util::DriveSignal(30, 30));
      } else {
        Intake::instance->setFromMacro(0);
        Drive::instance->setFromMacro(util::DriveSignal(0, 0));
      }
      if(std::fabs(pot->get_value() - constants::RobotConstants::TRAY_SCORE) < 900) {
        count_stop_states_++;
      }
      else {
        count_stop_states_ = 0;
      }
      if(count_stop_states_ && count_stop_states_ > 10) {
        setOpenLoop(0.0);
        scoring_state_ = 1;
      }
    }


  }
  void Tray::activateScore() {
    current_state = SCORE_TRAY;
    count_stop_states_ = 0;
    scoring_state_ = 0;
  }
  bool Tray::doneWithScore() {
    return scoring_state_;
  }
  ControlState Tray::getState() {
    return current_state;
  }
  double Tray::getPositionError() {
    return demand.getValue() -  pot->get_value();
  }
  double Tray::getMotorVelocity() {
    return motor->get_actual_velocity();
  }
  void Tray::tare() {
    motor->tare_position();
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
