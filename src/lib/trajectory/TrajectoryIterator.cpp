//
// Created by alexweiss on 8/3/19.
//

#include "TrajectoryIterator.hpp"
#include "../utility/Utility.hpp"
#include <stdio.h>

namespace trajectory {

  template <class S>
  TrajectoryIterator<S>::TrajectoryIterator(std::shared_ptr<TrajectoryView<S>> view) :
      current_sample_(S(), 0, 0)
  {
    view_ = view;
  }

  template <class S>
  void TrajectoryIterator<S>::setup() {
    current_sample_ = view_->sample(view_->first_interpolant());
    progress_ = view_->first_interpolant();
  }

  template <class S>
  bool TrajectoryIterator<S>::isDone() {
    return getRemainingProgress() == 0.0;
  }

  template <class S>
  double TrajectoryIterator<S>::getProgress() {
    return progress_;
  }

  template <class S>
  double TrajectoryIterator<S>::getRemainingProgress() {
    return MAX(0.0, view_->last_interpolant() - progress_);
  }

  template <class S>
  TrajectorySamplePoint<S> TrajectoryIterator<S>::getSample() {
    return current_sample_;
  }

  template <class S>
  S TrajectoryIterator<S>::getState() {
    return getSample().state();
  }

  template <class S>
  TrajectorySamplePoint<S> TrajectoryIterator<S>::advance(double additional_progress) {
    progress_ = MAX(view_->first_interpolant(),
                         MIN(view_->last_interpolant(), progress_ + additional_progress));  current_sample_ = view_->sample(progress_);
    return current_sample_;
  }

  template <class S>
  TrajectorySamplePoint<S> TrajectoryIterator<S>::preview(double additional_progress) {
    double progress = MAX(view_->first_interpolant(),
                               MIN(view_->last_interpolant(), progress_ + additional_progress));
    return view_->sample(progress);
  }

  template <class S>
  Trajectory<S> TrajectoryIterator<S>::trajectory(){
    return view_->trajectory();
  }

  template class TrajectoryIterator<geometry::Translation2d>;
  template class TrajectoryIterator<geometry::Rotation2d>;
  template class TrajectoryIterator<geometry::Pose2d>;
  template class TrajectoryIterator<geometry::Pose2dWithCurvature>;


}