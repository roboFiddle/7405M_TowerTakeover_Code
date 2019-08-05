//
// Created by alexweiss on 8/3/19.
//

#include "../utility/Utility.hpp"
#include "DistanceView.hpp"

namespace trajectory {

  template <class S>
  DistanceView<S>::DistanceView(Trajectory<S> trajectory) {
    trajectory_ = trajectory;
    distances_ = new double[trajectory_.size()];
    distances_[0] = 0.0;
    for (int i = 1; i < trajectory_.size(); ++i) {
      distances_[i] = distances_[i - 1] + trajectory_.getState(i - 1).distance(trajectory_.getState(i));
    }
  }

  template <class S>
  TrajectorySamplePoint<S> DistanceView<S>::sample(double distance) {
    if (distance >= last_interpolant())
      return TrajectorySamplePoint<S>(trajectory_.getPoint(trajectory_.length() - 1));
    if (distance <= 0.0)
      return TrajectorySamplePoint<S>(trajectory_.getPoint(0));
    for (int i = 1; i < trajectory_.size(); ++i) {
      TrajectoryPoint<S> s = trajectory_.getPoint(i);
      if (distances_[i] >= distance) {
        TrajectoryPoint<S> prev_s = trajectory_.getPoint(i - 1);
        if (FEQUALS(distances_[i], distances_[i - 1])) {
          return TrajectorySamplePoint<S>(s);
        } else {
          return TrajectorySamplePoint<S>(prev_s.state().interpolate(s.state(),
                                                                         (distance - distances_[i - 1]) / (distances_[i] - distances_[i - 1])), i - 1, i);
        }
      }
    }
  }

  template <class S>
  double DistanceView<S>::last_interpolant() {
    return distances_[trajectory_.size() - 1];
  }

  template <class S>
  double DistanceView<S>::first_interpolant() {
    return 0.0;
  }

  template <class S>
  Trajectory<S> DistanceView<S>::trajectory() {
    return trajectory_;
  }
}