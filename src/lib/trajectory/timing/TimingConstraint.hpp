//
// Created by alexweiss on 7/8/19.
//

#ifndef INC_7405M_CODE_SRC_LIB_TRAJECTORY_TIMING_TIMINGCONSTRAINT_HPP_
#define INC_7405M_CODE_SRC_LIB_TRAJECTORY_TIMING_TIMINGCONSTRAINT_HPP_

#include "../../geometry/interfaces/State.hpp"
#include <type_traits>
#include <cmath>

namespace trajectory {
  class MinMaxAcceleration {
   protected:
    double min_acceleration_;
    double max_acceleration_;

   public:
    MinMaxAcceleration();
    MinMaxAcceleration(double min_acceleration, double max_acceleration);
    double min_acceleration();
    double max_acceleration();
    bool valid();
    static MinMaxAcceleration kNoLimits ;
  };

  template <class T>
  class TimingConstraint {
  static_assert(std::is_base_of<geometry::State<T>, T>::value, "T is not derived from State");
   public:
    virtual double getMaxVelocity(T state) = 0;
    virtual MinMaxAcceleration getMinMaxAcceleration(T state, double velocity) = 0;
  };
}
#endif //INC_7405M_CODE_SRC_LIB_TRAJECTORY_TIMING_TIMINGCONSTRAINT_HPP_
