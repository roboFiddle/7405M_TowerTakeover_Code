//
// Created by alexweiss on 7/9/19.
//

#include "CentripetalAccelerationConstraint.hpp"
#include "../../physics/DifferentialDrive.hpp"
#include <cmath>

namespace trajectory {
  CentripetalAccelerationConstraint::CentripetalAccelerationConstraint(double max_centripetal_accel) {
    max_centripetal_accel_ = max_centripetal_accel;
  }
  double CentripetalAccelerationConstraint::getMaxVelocity(geometry::Pose2dWithCurvature state) {
    return std::sqrt(fabs(max_centripetal_accel_ / state.curvature()));
  }
physics::DifferentialDrive::MinMaxAcceleration CentripetalAccelerationConstraint::getMinMaxAcceleration(
      geometry::Pose2dWithCurvature state, double velocity) {
    return physics::DifferentialDrive::MinMaxAcceleration::kNoLimits;
  }
}