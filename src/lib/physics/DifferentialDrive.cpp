//
// Created by alexweiss on 7/4/19.
//

#include "DifferentialDrive.hpp"
#include "../utility/Utility.hpp"
#include <cmath>

namespace physics {
  DifferentialDrive::DifferentialDrive(double mass, double moi, double angular_drag,
                                       double wheel_radius, double effective_wheelbase_radius,
                                       DCMotorTransmission left_transmission, DCMotorTransmission right_transmission) :
  mass_ (mass),
  moi_ (moi),
  angular_drag_ (angular_drag),
  wheel_radius_ (wheel_radius),
  effective_wheelbase_radius_ (effective_wheelbase_radius),
  left_ (left_transmission),
  right_ (right_transmission) {
  }
  double DifferentialDrive::mass() {
    return mass_;
  }
  double DifferentialDrive::moi() {
    return moi_;
  }
  double DifferentialDrive::wheel_radius() {
    return wheel_radius_;
  }
  double DifferentialDrive::effective_wheelbase_radius() {
    return effective_wheelbase_radius_;
  }
  DCMotorTransmission* DifferentialDrive::left_transmission() {
    return &left_;
  }
  DCMotorTransmission* DifferentialDrive::right_transmission() {
    return &right_;
  }

  DifferentialDrive::ChassisState DifferentialDrive::solveForwardKinematics(DifferentialDrive::WheelState wheel_motion) {
    ChassisState chassis_motion;
    chassis_motion.linear_ = wheel_radius_ * (wheel_motion.right_ + wheel_motion.left_) / 2.0;
    chassis_motion.angular_ = wheel_radius_ * (wheel_motion.right_ - wheel_motion.left_) / (2.0 *
        effective_wheelbase_radius_);
    return chassis_motion;
  }
  DifferentialDrive::WheelState DifferentialDrive::solveInverseKinematics(DifferentialDrive::ChassisState chassis_motion) {
    WheelState wheel_motion;
    wheel_motion.left_ = (chassis_motion.linear_ - effective_wheelbase_radius_ * chassis_motion.angular_) /
        wheel_radius_;
    wheel_motion.right_ = (chassis_motion.linear_ + effective_wheelbase_radius_ * chassis_motion.angular_) /
        wheel_radius_;
    return wheel_motion;
  }

  DifferentialDrive::DriveDynamics DifferentialDrive::solveForwardDynamics(DifferentialDrive::ChassisState chassis_velocity, DifferentialDrive::WheelState voltage) {
    DriveDynamics dynamics;
    dynamics.wheel_velocity = solveInverseKinematics(chassis_velocity);
    dynamics.chassis_velocity = chassis_velocity;
    dynamics.curvature = dynamics.chassis_velocity.angular_ / dynamics.chassis_velocity.linear_;
    if (std::isnan(dynamics.curvature))
      dynamics.curvature = 0.0;
    dynamics.voltage = voltage;
    solveForwardDynamics(&dynamics);
    return dynamics;
  }
  DifferentialDrive::DriveDynamics DifferentialDrive::solveForwardDynamics(DifferentialDrive::WheelState wheel_velocity, DifferentialDrive::WheelState voltage) {
    DriveDynamics dynamics;
    dynamics.wheel_velocity = wheel_velocity;
    dynamics.chassis_velocity = solveForwardKinematics(wheel_velocity);
    dynamics.curvature = dynamics.chassis_velocity.angular_ / dynamics.chassis_velocity.linear_;
    if (std::isnan(dynamics.curvature))
      dynamics.curvature = 0.0;
    dynamics.voltage = voltage;
    solveForwardDynamics(&dynamics);
    return dynamics;
  }

  void DifferentialDrive::solveForwardDynamics(DifferentialDrive::DriveDynamics* dynamics) {
    bool left_stationary = FEQUALS(dynamics->wheel_velocity.left_, 0.0) && fabs(dynamics->voltage.left_) < left_.friction_voltage();
    bool right_stationary = FEQUALS(dynamics->wheel_velocity.right_, 0.0) && fabs(dynamics->voltage.right_) < right_.friction_voltage();
    if (left_stationary && right_stationary) {
      // Neither side breaks static friction, so we remain stationary.
      dynamics->wheel_torque.left_ = dynamics->wheel_torque.right_ = 0.0;
      dynamics->chassis_acceleration.linear_ = dynamics->chassis_acceleration.angular_ = 0.0;
      dynamics->wheel_acceleration.left_ = dynamics->wheel_acceleration.right_ = 0.0;
      dynamics->dcurvature = 0.0;
      return;
    }

    // Solve for motor torques generated on each side.

    dynamics->wheel_torque.left_ = left_.get_torque_at_voltage(dynamics->wheel_velocity.left_, dynamics->voltage.left_);
    dynamics->wheel_torque.right_ = right_.get_torque_at_voltage(dynamics->wheel_velocity.right_, dynamics->voltage.right_);

    // Add forces and torques about the center of mass.
    dynamics->chassis_acceleration.linear_ = (dynamics->wheel_torque.right_ + dynamics->wheel_torque.left_) / (wheel_radius_ * mass_);
    // (Tr - Tl) / r_w * r_wb - drag * w = I * angular_accel
    dynamics->chassis_acceleration.angular_ = effective_wheelbase_radius_ * (dynamics->wheel_torque.right_ - dynamics->wheel_torque.left_) / (wheel_radius_ * moi_) - dynamics->chassis_velocity.angular_ * angular_drag_ / moi_;

    // Solve for change in curvature from angular_ acceleration.
    // total angular_ accel = linear_accel * curvature + v^2 * dcurvature
    dynamics->dcurvature = (dynamics->chassis_acceleration.angular_ - dynamics->chassis_acceleration.linear_ * dynamics->curvature) / (dynamics->chassis_velocity.linear_ * dynamics->chassis_velocity.linear_);
    if (std::isnan(dynamics->dcurvature))
      dynamics->dcurvature = 0.0;

    // Resolve chassis accelerations to each wheel.
    dynamics->wheel_acceleration.left_ = dynamics->chassis_acceleration.linear_ - dynamics->chassis_acceleration.angular_ * effective_wheelbase_radius_;
    dynamics->wheel_acceleration.right_ = dynamics->chassis_acceleration.linear_ + dynamics->chassis_acceleration.angular_ * effective_wheelbase_radius_;
  }

  DifferentialDrive::DriveDynamics DifferentialDrive::solveInverseDynamics(DifferentialDrive::ChassisState chassis_velocity, DifferentialDrive::ChassisState chassis_acceleration) {
    DriveDynamics dynamics;
    dynamics.chassis_velocity = chassis_velocity;
    dynamics.curvature = dynamics.chassis_velocity.angular_ / dynamics.chassis_velocity.linear_;
    if (std::isnan(dynamics.curvature))
      dynamics.curvature = 0.0;
    dynamics.chassis_acceleration = chassis_acceleration;
    dynamics.dcurvature = (dynamics.chassis_acceleration.angular_ - dynamics.chassis_acceleration.linear_ * dynamics.curvature) / (dynamics.chassis_velocity.linear_ * dynamics.chassis_velocity.linear_);
    if (std::isnan(dynamics.dcurvature))
      dynamics.dcurvature = 0.0;
    dynamics.wheel_velocity = solveInverseKinematics(chassis_velocity);
    dynamics.wheel_acceleration = solveInverseKinematics(chassis_acceleration);
    solveInverseDynamics(&dynamics);
    return dynamics;
  }
  DifferentialDrive::DriveDynamics DifferentialDrive::solveInverseDynamics(DifferentialDrive::WheelState wheel_velocity, DifferentialDrive::WheelState wheel_acceleration) {
    DriveDynamics dynamics;
    dynamics.chassis_velocity = solveForwardKinematics(wheel_velocity);
    dynamics.curvature = dynamics.chassis_velocity.angular_ / dynamics.chassis_velocity.linear_;
    if (std::isnan(dynamics.curvature))
      dynamics.curvature = 0.0;
    dynamics.chassis_acceleration = solveForwardKinematics(wheel_acceleration);
    dynamics.dcurvature = (dynamics.chassis_acceleration.angular_ - dynamics.chassis_acceleration.linear_ * dynamics.curvature) / (dynamics.chassis_velocity.linear_ * dynamics.chassis_velocity.linear_);
    if (std::isnan(dynamics.dcurvature))
      dynamics.dcurvature = 0.0;
    dynamics.wheel_velocity = wheel_velocity;
    dynamics.wheel_acceleration = wheel_acceleration;
    solveInverseDynamics(&dynamics);
    return dynamics;
  }
  void DifferentialDrive::solveInverseDynamics(DifferentialDrive::DriveDynamics* dynamics) {
    dynamics->wheel_torque.left_ = wheel_radius_ / 2.0 * (dynamics->chassis_acceleration.linear_ * mass_ - dynamics->chassis_acceleration.angular_ * moi_ / effective_wheelbase_radius_ -
        dynamics->chassis_velocity.angular_ * angular_drag_ / effective_wheelbase_radius_);
    dynamics->wheel_torque.right_ = wheel_radius_ / 2.0 * (dynamics->chassis_acceleration.linear_ * mass_ +
        dynamics->chassis_acceleration.angular_ * moi_ / effective_wheelbase_radius_ +
        dynamics->chassis_velocity.angular_ * angular_drag_ / effective_wheelbase_radius_);

    // Solve for input voltages.
    dynamics->voltage.left_ = left_.get_voltage_for_torque(dynamics->wheel_velocity.left_, dynamics->wheel_torque.left_);
    dynamics->voltage.right_ = right_.get_voltage_for_torque(dynamics->wheel_velocity.right_, dynamics->wheel_torque.right_);
  }
  double DifferentialDrive::getMaxAbsVelocity(double curvature, double max_abs_voltage) {
    double left_speed_at_max_voltage = left_.free_speed_at_voltage(max_abs_voltage);
    double right_speed_at_max_voltage = right_.free_speed_at_voltage(max_abs_voltage);

    if(FEQUALS(curvature, 0.0)) {
      return wheel_radius_ * std::fmin(left_speed_at_max_voltage, right_speed_at_max_voltage);
    }

    if(std::isinf(curvature)) {
      double wheel_speed = std::fmin(left_speed_at_max_voltage, right_speed_at_max_voltage); // Turn in place.  Return value meaning becomes angular velocity.
      return std::copysign(1.0, curvature) * wheel_radius_ * wheel_speed / effective_wheelbase_radius_;
    }

    double right_speed_if_left_max = left_speed_at_max_voltage * (effective_wheelbase_radius_ * curvature + 1.0) / (1.0 - effective_wheelbase_radius_ * curvature);
    if (fabs(right_speed_if_left_max) <= right_speed_at_max_voltage + EPSILON) {
      return wheel_radius_ * (left_speed_at_max_voltage + right_speed_if_left_max) / 2.0; // Left max is active constraint.
    }

    double left_speed_if_right_max = right_speed_at_max_voltage * (1.0 - effective_wheelbase_radius_ * curvature) / (1.0 + effective_wheelbase_radius_ * curvature);
    return wheel_radius_ * (right_speed_at_max_voltage + left_speed_if_right_max) / 2.0; // Right at max is active constraint.
  }

  DifferentialDrive::MinMax DifferentialDrive::getMinMaxAcceleration(DifferentialDrive::ChassisState chassis_velocity, double curvature, double max_abs_voltage) {
    MinMax result;
    WheelState wheel_velocities = solveInverseKinematics(chassis_velocity);
    result.min_ = INFINITY;
    result.max_ = -INFINITY;

    // Math:
    // (Tl + Tr) / r_w = m*a
    // (Tr - Tl) / r_w * r_wb - drag*w = i*(a * k + v^2 * dk)

    // 2 equations, 2 unknowns.
    // Solve for a and (Tl|Tr)

    double linear_term = std::isinf(curvature) ? 0.0 : mass_ * effective_wheelbase_radius_;
    double angular_term = std::isinf(curvature) ? moi_ : moi_ * curvature;
    double drag_torque = chassis_velocity.angular_ * angular_drag_;

    for(bool left : {0, 1}) {
      for(double sign : {-1.0, 1.0}) {

        DCMotorTransmission* fixed_transmission = left ? &left_ : &right_;
        DCMotorTransmission* variable_transmission = left ? &right_ : &left_;
        double fixed_torque = fixed_transmission->get_torque_at_voltage(wheel_velocities.get(left), sign * max_abs_voltage);
        double variable_torque = 0.0;

        //TODO IMPORTANT: CALCULATE VARIABLE TORQUE -- CURRENTLY WRONG
        if (left) {
          variable_torque = -drag_torque * mass_ * wheel_radius_ + fixed_torque *
              (linear_term + angular_term)) / (linear_term - angular_term);
        } else {
          variable_torque = drag_torque * mass_ * wheel_radius_ + fixed_torque *
              (linear_term - angular_term)) / (linear_term + angular_term);
        }

        double variable_voltage = variable_transmission->get_voltage_for_torque(wheel_velocities.get(!left), variable_torque);
        if (fabs(variable_voltage) <= max_abs_voltage + EPSILON) {
          double accel = 0.0;
          if (std::isinf(curvature)) {
            accel = (left ? -1.0 : 1.0) * (fixed_torque - variable_torque) * effective_wheelbase_radius_
                / (moi_ * wheel_radius_) - drag_torque / moi_;
          } else {
            accel = (fixed_torque + variable_torque) / (mass_ * wheel_radius_);
          }
          result.min_ = std::fmin(result.min_, accel);
          result.max_ = std::fmax(result.max_, accel);
        }

      }
    }

    return result;

  }

}