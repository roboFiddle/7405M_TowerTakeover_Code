//
// Created by alexweiss on 7/8/19.
//

#include "TimingConstraint.hpp"

namespace trajectory {
  template <class T> TimingConstraint<T>::MinMaxAcceleration::MinMaxAcceleration() {
    // No limits.
    min_acceleration_ = -INFINITY;
    max_acceleration_ = INFINITY;
  }
  template <class T> TimingConstraint<T>::MinMaxAcceleration::MinMaxAcceleration(double min_acceleration, double max_acceleration) {
    min_acceleration_ = min_acceleration;
    max_acceleration_ = max_acceleration;
  }

  template <class T> double TimingConstraint<T>::MinMaxAcceleration::min_acceleration() {
    return min_acceleration_;
  }

  template <class T> double TimingConstraint<T>::MinMaxAcceleration::max_acceleration() {
    return max_acceleration_;
  }

  template <class T> bool TimingConstraint<T>::MinMaxAcceleration::valid() {
    return min_acceleration() <= max_acceleration();
  }
}