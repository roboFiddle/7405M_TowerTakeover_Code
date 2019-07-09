//
// Created by alexweiss on 7/9/19.
//

#include "CentripetalAccelerationConstraint.hpp"
#include <cmath>

namespace trajectory {
  CentripetalAccelerationConstraint::CentripetalAccelerationConstraint(double max_centripetal_accel) {
    max_centripetal_accel_ = max_centripetal_accel;
  }
  double CentripetalAccelerationConstraint::getMaxVelocity(geometry::Pose2dWithCurvature state) {
    return std::sqrt(fabs(max_centripetal_accel_ / state.curvature()));
  }
  MinMaxAcceleration CentripetalAccelerationConstraint::getMinMaxAcceleration(
      geometry::Pose2dWithCurvature state, double velocity) {
    return MinMaxAcceleration::kNoLimits;
  }
}