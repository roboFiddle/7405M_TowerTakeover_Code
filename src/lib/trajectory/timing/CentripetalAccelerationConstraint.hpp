//
// Created by alexweiss on 7/9/19.
//

#ifndef INC_7405M_CODE_SRC_LIB_TRAJECTORY_TIMING_CENTRIPETALACCELERATIONCONSTRAINT_HPP_
#define INC_7405M_CODE_SRC_LIB_TRAJECTORY_TIMING_CENTRIPETALACCELERATIONCONSTRAINT_HPP_

#include "TimingConstraint.hpp"
#include "../../geometry/Pose2dWithCurvature.hpp"

namespace trajectory {
  class CentripetalAccelerationConstraint : TimingConstraint<geometry::Pose2dWithCurvature> {
   private:
    double max_centripetal_accel_;
   public:
    CentripetalAccelerationConstraint(double max_centripetal_accel);
    double getMaxVelocity(geometry::Pose2dWithCurvature state);
    MinMaxAcceleration getMinMaxAcceleration(geometry::Pose2dWithCurvature state, double velocity);
  };
}
#endif //INC_7405M_CODE_SRC_LIB_TRAJECTORY_TIMING_CENTRIPETALACCELERATIONCONSTRAINT_HPP_
