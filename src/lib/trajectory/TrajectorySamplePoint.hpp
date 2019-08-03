//
// Created by alexweiss on 8/3/19.
//

#ifndef INC_7405M_CODE_SRC_LIB_TRAJECTORY_TRAJECTORYSAMPLEPOINT_HPP_
#define INC_7405M_CODE_SRC_LIB_TRAJECTORY_TRAJECTORYSAMPLEPOINT_HPP_

#include <type_traits>
#include "../geometry/interfaces/State.hpp"
#include "TrajectoryPoint.hpp"

namespace trajectory {
  template <class S>
  class TrajectorySamplePoint {
    static_assert(std::is_base_of<geometry::State<S>, S>::value, "S must derive from State");
    protected:
      S state_;
      int index_floor_;
      int index_ceil_;
    public:
      TrajectorySamplePoint(S state, int index_floor, int index_ceil);
      TrajectorySamplePoint(TrajectoryPoint<S> point);
      S state();
      int index_floor();
      int index_ceil();
  };
}

#endif //INC_7405M_CODE_SRC_LIB_TRAJECTORY_TRAJECTORYSAMPLEPOINT_HPP_
