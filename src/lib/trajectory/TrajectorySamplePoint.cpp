//
// Created by alexweiss on 8/3/19.
//

#include "TrajectorySamplePoint.hpp"


namespace trajectory {
  template <class S>
  TrajectorySamplePoint<S>::TrajectorySamplePoint(S state, int index_floor, int index_ceil) {
    state_ = state;
    index_floor_ = index_floor;
    index_ceil_ = index_ceil;
  }

  template <class S>
  TrajectorySamplePoint<S>::TrajectorySamplePoint(TrajectoryPoint<S> point) {
    state_ = point.state();
    index_floor_ = index_ceil_ = point.index();
  }

  template <class S>
  S TrajectorySamplePoint<S>::state() {
    return state_;
  }

  template <class S>
  int TrajectorySamplePoint<S>::index_floor() {
    return index_floor_;
  }

  template <class S>
  int TrajectorySamplePoint<S>::index_ceil() {
    return index_ceil_;
  }

  template class TrajectorySamplePoint<geometry::Translation2d>;
  template class TrajectorySamplePoint<geometry::Rotation2d>;
  template class TrajectorySamplePoint<geometry::Pose2d>;
  template class TrajectorySamplePoint<geometry::Pose2dWithCurvature>;

}