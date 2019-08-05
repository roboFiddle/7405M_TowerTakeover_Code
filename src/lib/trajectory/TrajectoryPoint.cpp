//
// Created by alexweiss on 8/3/19.
//

#include "TrajectoryPoint.hpp"

namespace trajectory {

  template <class S>
  TrajectoryPoint<S>::TrajectoryPoint(S state, int index) {
    state_ = state;
    index_ = index;
  }

  template <class S>
  S TrajectoryPoint<S>::state() {
    return state_;
  }

  template <class S>
  int TrajectoryPoint<S>::index() {
    return index_;
  }
}