//
// Created by alexweiss on 8/3/19.
//

#include "TimedView.hpp"
#include "../utility/Utility.hpp"

namespace trajectory {

  template <class S>
  TimedView<S>::TimedView(Trajectory<TimedState<S>> trajectory) {
    trajectory_ = trajectory;
    start_t_ = trajectory_.getState(0).t();
    end_t_ = trajectory_.getState(trajectory_.length() - 1).t();
  }

  template <class S>
  double TimedView<S>::first_interpolant() {
    return start_t_;
  }

  template <class S>
  double TimedView<S>::last_interpolant() {
    return end_t_;
  }

  template <class S>
  TrajectorySamplePoint<TimedState<S>> TimedView<S>::sample(double t) {
    if (t >= end_t_) {
      return TrajectorySamplePoint<TimedState<S>>(trajectory_.getPoint(trajectory_.length() - 1));
    }
    if (t <= start_t_) {
      return TrajectorySamplePoint<TimedState<S>>(trajectory_.getPoint(0));
    }
    for (int i = 1; i < trajectory_.length(); ++i) {
      TrajectoryPoint<TimedState<S>> s = trajectory_.getPoint(i);
      if (s.state().t() >= t) {
        TrajectoryPoint<TimedState<S>> prev_s = trajectory_.getPoint(i - 1);
        if (FEQUALS(s.state().t(), prev_s.state().t())) {
          return TrajectorySamplePoint<TimedState<S>>(s);
        }
        return TrajectorySamplePoint<TimedState<S>>(prev_s.state().interpolate(s.state(),
                                                                      (t - prev_s.state().t()) / (s.state().t() - prev_s.state().t())), i - 1, i);
      }
    }
  }

  template <class S>
  Trajectory<TimedState<S>> TimedView<S>::trajectory() {
    return trajectory_;
  }


  template class TimedView<geometry::Translation2d>;
  template class TimedView<geometry::Rotation2d>;
  template class TimedView<geometry::Pose2d>;
  template class TimedView<geometry::Pose2dWithCurvature>;


}