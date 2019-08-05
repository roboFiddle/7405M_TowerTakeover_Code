//
// Created by alexweiss on 8/3/19.
//

#ifndef INC_7405M_CODE_SRC_LIB_TRAJECTORY_TRAJECTORYVIEW_HPP_
#define INC_7405M_CODE_SRC_LIB_TRAJECTORY_TRAJECTORYVIEW_HPP_

#include <type_traits>
#include "../geometry/interfaces/State.hpp"
#include "Trajectory.hpp"
#include "TrajectorySamplePoint.hpp"

namespace trajectory {

  template<class S> class Trajectory;

  template <class S>
  class TrajectoryView {
    static_assert(std::is_base_of<geometry::State<S>, S>::value, "S must derive from State");
    TrajectorySamplePoint<S> sample(double interpolant);
    double first_interpolant();
    double last_interpolant();
    Trajectory<S> trajectory();
  };
}

#endif //INC_7405M_CODE_SRC_LIB_TRAJECTORY_TRAJECTORYVIEW_HPP_
