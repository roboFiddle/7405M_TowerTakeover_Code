//
// Created by alexweiss on 7/9/19.
//

#include "VelocityLimitRegionConstraint.hpp"
#include <cmath>

namespace trajectory {
  template<class S> VelocityLimitRegionConstraint<S>::VelocityLimitRegionConstraint(geometry::Translation2d min_corner,
                                                                                 geometry::Translation2d max_corner, double velocity_limit) {
    min_corner_ = min_corner;
    max_corner_ = max_corner;
    velocity_limit_ = velocity_limit;
  }
  template<class S> double VelocityLimitRegionConstraint<S>::getMaxVelocity(S state) {
    geometry::Translation2d translation = state.translation();
    if (translation.x() <= max_corner_.x() && translation.x() >= min_corner_.x() &&
        translation.y() <= max_corner_.y() && translation.y() >= min_corner_.y()) {
      return velocity_limit_;
    }
    return INFINITY;
  }
  template<class S> physics::DifferentialDrive::MinMaxAcceleration VelocityLimitRegionConstraint<S>::getMinMaxAcceleration(S state, double velocity) {
    return physics::DifferentialDrive::MinMaxAcceleration::kNoLimits;
  }

  template class VelocityLimitRegionConstraint<geometry::Translation2d>;


}