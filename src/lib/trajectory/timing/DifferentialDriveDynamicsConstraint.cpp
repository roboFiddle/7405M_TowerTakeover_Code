//
// Created by alexweiss on 7/9/19.
//

#include "DifferentialDriveDynamicsConstraint.hpp"
#include "../../utility/Units.hpp"

namespace trajectory {
  template<class S> DifferentialDriveDynamicsConstraint<S>::DifferentialDriveDynamicsConstraint(
    physics::DifferentialDrive* drive, double abs_voltage_limit) {
    drive_ = drive;
    abs_voltage_limit_ = abs_voltage_limit;

  }
  template<class S> double DifferentialDriveDynamicsConstraint<S>::getMaxVelocity(S* state) {
    double curvature_in_inverse_meters = (state->curvature() * units::metre).Convert(units::inch);
    double max_velo_in_meters = drive_->getMaxAbsVelocity(curvature_in_inverse_meters, abs_voltage_limit_);
    return (max_velo_in_meters * units::metre).Convert(units::inch);
  }
  template<class S> physics::DifferentialDrive::MinMaxAcceleration DifferentialDriveDynamicsConstraint<S>::getMinMaxAcceleration(S state, double velocity) {
    double velo_in_meters = (velocity * units::inch).Convert(units::metre);
    double curvature_in_inverse_meters = (state->curvature() * units::metre).Convert(units::inch);

    physics::DifferentialDrive::MinMaxAcceleration min_max = drive_->getMinMaxAcceleration(physics::DifferentialDrive::ChassisState(
        velo_in_meters, state->curvature() * velocity), curvature_in_inverse_meters,
        abs_voltage_limit_); // Curvature is in inverse inches, so meters_to_inches is correct.

    return min_max;
  }
}