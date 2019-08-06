//
// Created by alexweiss on 8/3/19.
//

#include "../utility/Utility.hpp"
#include "DistanceView.hpp"
#include "../geometry/Translation2d.hpp"
#include "../geometry/Rotation2d.hpp"
#include "../geometry/Pose2d.hpp"
#include "../geometry/Pose2dWithCurvature.hpp"

namespace trajectory {

  template <class S>
  DistanceView<S>::DistanceView(Trajectory<S>* trajectory) {
    trajectory_ = trajectory;
    distances_.resize(trajectory->length(), 0.0);
    distances_[0] = 0.0;
    for (int i = 1; i < trajectory_->length(); ++i) {
      distances_[i] = distances_[i - 1] + trajectory_->getState(i - 1).distance(trajectory_->getState(i));
    }
  }

  template <class S>
  TrajectorySamplePoint<S> DistanceView<S>::sample(double distance) {
    if (distance >= last_interpolant())
      return TrajectorySamplePoint<S>(trajectory_->getPoint(trajectory_->length() - 1));
    if (distance <= 0.0)
      return TrajectorySamplePoint<S>(trajectory_->getPoint(0));
    for (int i = 1; i < trajectory_->length(); ++i) {
      TrajectoryPoint<S> s = trajectory_->getPoint(i);
      if (distances_[i] >= distance) {
        TrajectoryPoint<S> prev_s = trajectory_->getPoint(i - 1);
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
    return distances_[trajectory_->length() - 1];
  }

  template <class S>
  double DistanceView<S>::first_interpolant() {
    return 0.0;
  }

  template <class S>
  Trajectory<S> DistanceView<S>::trajectory() {
    return *trajectory_;
  }


  template class DistanceView<geometry::Translation2d>;
  template class DistanceView<geometry::Rotation2d>;
  template class DistanceView<geometry::Pose2d>;
  template class DistanceView<geometry::Pose2dWithCurvature>;

}