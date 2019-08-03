//
// Created by alexweiss on 8/2/19.
//

#ifndef INC_7405M_CODE_SRC_LIB_TRAJECTORY_TIMING_TIMINGUTIL_HPP_
#define INC_7405M_CODE_SRC_LIB_TRAJECTORY_TIMING_TIMINGUTIL_HPP_

#include <list>
#include <string>
#include <type_traits>

#include "../../geometry/interfaces/State.hpp"
#include "../Trajectory.hpp"
#include "TimingConstraint.hpp"
#include "TimedState.hpp"

namespace trajectory {
  class TimingUtil {
   public:

    template<class S>
    static Trajectory<TimedState<S>> timeParameterizeTrajectory(
        bool reverse,
        DistanceView<S> distance_view,
        double step_size,
        std::list<TimingConstraint<S>> constraints,
        double start_velocity,
        double end_velocity,
        double max_velocity,
        double max_abs_acceleration);

    template<class S>
    static Trajectory<TimedState<S>> timeParameterizeTrajectory(
        bool reverse,
        std::list<S> states,
        std::list<TimingConstraint<S>> constraints,
        double start_velocity,
        double end_velocity,
        double max_velocity,
        double max_abs_acceleration);

    template<class S>
    class ConstrainedState : public S {
      static_assert(std::is_base_of<geometry::State<S>, S>::value, "S is not derived from State");
     public:
      double distance_;
      double max_velocity_;
      double min_acceleration_;
      double max_acceleration_;
    };

  };
}
#endif //INC_7405M_CODE_SRC_LIB_TRAJECTORY_TIMING_TIMINGUTIL_HPP_
