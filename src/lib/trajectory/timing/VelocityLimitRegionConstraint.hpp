//
// Created by alexweiss on 7/9/19.
//

#ifndef INC_7405M_CODE_SRC_LIB_TRAJECTORY_TIMING_VELOCITYLIMITREGIONCONSTRAINT_HPP_
#define INC_7405M_CODE_SRC_LIB_TRAJECTORY_TIMING_VELOCITYLIMITREGIONCONSTRAINT_HPP_

#include "TimingConstraint.hpp"
#include "../../geometry/interfaces/ITranslation2d.hpp"
#include "../../geometry/Translation2d.hpp"
#include <type_traits>

namespace trajectory {
  template<class S>
  class VelocityLimitRegionConstraint : TimingConstraint<S> {
    static_assert(std::is_base_of<geometry::ITranslation2d < S > , S > ::value, "S must derive from ITranslation2d<S>");
   protected:
    geometry::Translation2d min_corner_;
    geometry::Translation2d max_corner_;
    double velocity_limit_;
   public:
    VelocityLimitRegionConstraint(geometry::Translation2d min_corner, geometry::Translation2d max_corner, double velocity_limit);
    double getMaxVelocity(S state);
    MinMaxAcceleration getMinMaxAcceleration(S state, double velocity);

  };
}
#endif //INC_7405M_CODE_SRC_LIB_TRAJECTORY_TIMING_VELOCITYLIMITREGIONCONSTRAINT_HPP_
