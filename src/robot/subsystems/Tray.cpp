#include "Tray.hpp"
#include "Drive.hpp"
#include "Intake.hpp"
#include "../Constants.hpp"

#include <stdio.h>

namespace subsystems {
  Tray::TrayManager Tray::instance;

  Tray::Tray() {
    motor = new pros::Motor(constants::RobotConstants::motor_tray, pros::E_MOTOR_GEARSET_36, true, pros::E_MOTOR_ENCODER_DEGREES);
    pot = new pros::ADIAnalogIn(8);
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
    current_state = ControlState::POSITION_CONTROL;
    demand = control;
    limit_velo_ = 0;
  }
  void Tray::setPosition(units::Number control, bool limit_velo) {
    current_state = ControlState::POSITION_CONTROL;
    demand = control;
    limit_velo_ = limit_velo;
  }
  double Tray::getMultiplier() {
    if(!limit_velo_)
      return 1.5;

    if(pot->get_value() < 1400) {
      return 1;
    }
    else if(pot->get_value() > 3200) {
      return .1;
    }
    else {
      return 1 - (pot->get_value() - 1300) * (0.9/1900);
      //2/22 changed from -1400 to -1330  because tray was going too far forwards when stacking
    }
  }
  double Tray::get_position() {
    return pot->get_value();
  }
  void Tray::runPID(bool l = false) {
    double error = demand.getValue() - pot->get_value();
    double m = 0.1;
    printf("TRAY PID %f %f\n", demand.getValue(), 1.0*pot->get_value());
    if(demand.getValue() > 700)
      motor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    else
      motor->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    motor->move_velocity(error * m * (l ? 0.5 : 1));
  }
  void Tray::updateOutputs() {
    if(current_state == ControlState::OPEN_LOOP) {
      //printf("TRAY M %f\n", getMultiplier());
      motor->move_velocity(demand.getValue() * (demand.getValue() > 0 ? getMultiplier() : 1));
      if(pot->get_value() > 700)
        motor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      else
        motor->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    }
    else if(current_state == ControlState::POSITION_CONTROL)
      runPID();
    else if(current_state == ControlState::SCORE_TRAY) {
      limit_velo_ = true;
      demand = constants::RobotConstants::TRAY_SCORE;
      if(pot->get_value() < 3135) {
        motor->move_velocity(score_multi * getMultiplier());
      }
      else {
        motor->move_velocity(0);
      }
      //if(std::fabs(pot->get_value() - constants::RobotConstants::TRAY_SCORE) < 350) {
      if(pot->get_value() > 900 && pot->get_value() < 2600) {
        Intake::instance->setFromMacro(-intake_speed_);
      } else {
        Intake::instance->setFromMacro(0);
      }
      if(std::fabs(pot->get_value() - constants::RobotConstants::TRAY_SCORE) < 250) {
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
  void Tray::activateScore(double m, int s) {
    current_state = SCORE_TRAY;
    count_stop_states_ = 0;
    scoring_state_ = 0;
    score_multi = m;
    intake_speed_ = s;
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
