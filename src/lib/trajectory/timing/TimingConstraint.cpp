//
// Created by alexweiss on 7/8/19.
//

#include "TimingConstraint.hpp"

namespace trajectory {
  MinMaxAcceleration::MinMaxAcceleration() {
    // No limits.
    min_acceleration_ = -INFINITY;
    max_acceleration_ = INFINITY;
  }
  MinMaxAcceleration::MinMaxAcceleration(double min_acceleration, double max_acceleration) {
    min_acceleration_ = min_acceleration;
    max_acceleration_ = max_acceleration;
  }

  double MinMaxAcceleration::min_acceleration() {
    return min_acceleration_;
  }

  double MinMaxAcceleration::max_acceleration() {
    return max_acceleration_;
  }
  bool MinMaxAcceleration::valid() {
    return min_acceleration() <= max_acceleration();
  }
}