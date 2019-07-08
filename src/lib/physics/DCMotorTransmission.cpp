//
// Created by alexweiss on 7/4/19.
//

#include "DCMotorTransmission.hpp"
#include "../utility/Utility.hpp"

namespace physics {
  DCMotorTransmission::DCMotorTransmission(double speed_per_volt, double torque_per_volt, double friction_voltage)  :
      speed_per_volt_(speed_per_volt),
      torque_per_volt_(torque_per_volt),
      friction_voltage_(friction_voltage) {

  }
  double DCMotorTransmission::speed_per_volt() {
    return speed_per_volt_;
  }
  double DCMotorTransmission::torque_per_volt() {
    return torque_per_volt_;
  }
  double DCMotorTransmission::friction_voltage() {
    return friction_voltage_;
  }
  double DCMotorTransmission::free_speed_at_voltage(double voltage) {
    if (voltage > 0 && voltage - friction_voltage() > 0) {
      return (voltage - friction_voltage()) * speed_per_volt();
    } else if (voltage < 0 && voltage + friction_voltage() < 0) {
      return (voltage + friction_voltage()) * speed_per_volt();
    } else {
      return 0.0;
    }
  }
  double DCMotorTransmission::get_torque_at_voltage(double speed, double voltage) {
    double effective_voltage = voltage;
    if (speed > 0) {
      effective_voltage -= friction_voltage(); // Forward motion, rolling friction.
    } else if (speed < -0) {
      effective_voltage += friction_voltage(); // Reverse motion, rolling friction.
    } else if (voltage > 0) {
      effective_voltage = MAX(0.0, voltage - friction_voltage()); // System is static, forward torque.
    } else if (voltage < -0) {
      effective_voltage = MIN(0.0, voltage + friction_voltage()); // System is static, reverse torque.
    } else {
      return 0.0; // System is idle.
    }
    return torque_per_volt() * (-speed / speed_per_volt() + effective_voltage);
  }
  double DCMotorTransmission::get_voltage_for_torque(double speed, double torque) {
    double friction_voltage_offset;
    if (speed > 0) {
      friction_voltage_offset = friction_voltage(); // Forward motion, rolling friction.
    } else if (speed < -0) {
      friction_voltage_offset = -friction_voltage(); // Reverse motion, rolling friction.
    } else if (torque > 0) {
      friction_voltage_offset = friction_voltage(); // System is static, forward torque.
    } else if (torque < -0) {
      friction_voltage_offset = -friction_voltage(); // System is static, reverse torque.
    } else {
      return 0.0; // System is idle.
    }
    return torque / torque_per_volt() + speed / speed_per_volt() + friction_voltage_offset;
  }
}