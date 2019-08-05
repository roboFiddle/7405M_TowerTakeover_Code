//
// Created by alexweiss on 8/3/19.
//

#ifndef INC_7405M_CODE_SRC_LIB_TRAJECTORY_DISTANCEVIEW_HPP_
#define INC_7405M_CODE_SRC_LIB_TRAJECTORY_DISTANCEVIEW_HPP_

#include <type_traits>
#include "../geometry/interfaces/State.hpp"
#include "timing/TimedState.hpp"
#include "TrajectoryView.hpp"

namespace trajectory {

  template <class S>
  class DistanceView {
    static_assert(std::is_base_of<geometry::State<S>, S>::value, "S must derive from State");
    protected:
      Trajectory<S> trajectory_;
      double distances_[];
    public:
      DistanceView(Trajectory<S> trajectory);
      TrajectorySamplePoint<S> sample(double distance);
      double last_interpolant();
      double first_interpolant();
      Trajectory<S> trajectory();

  };
}

#endif //INC_7405M_CODE_SRC_LIB_TRAJECTORY_DISTANCEVIEW_HPP_
