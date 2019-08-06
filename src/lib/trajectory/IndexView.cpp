//
// Created by alexweiss on 8/4/19.
//

#include "IndexView.hpp"
#include "../geometry/Translation2d.hpp"
#include "../geometry/Rotation2d.hpp"
#include "../geometry/Pose2d.hpp"
#include "../geometry/Pose2dWithCurvature.hpp"
#include <stdio.h>

namespace trajectory {

  template<class T>
  IndexView<T>::IndexView(Trajectory<T>* x) : pointer_to_(x) {

  }

  template<class T>
  TrajectorySamplePoint<T> IndexView<T>::sample(double index) {
    return pointer_to_->getInterpolated(index);
  }

  template<class T>
  double IndexView<T>::last_interpolant() {
    printf("lmao this sucks %f\n", pointer_to_->length());
    return pointer_to_->length() == 0 ? 0 : pointer_to_->length() - 1;
  }

  template<class T>
  double IndexView<T>::first_interpolant() {
    return 0.0;
  }

  template<class T>
  Trajectory<T> IndexView<T>::trajectory() {
    return *pointer_to_;
  }

  template class IndexView<geometry::Translation2d>;
  template class IndexView<geometry::Rotation2d>;
  template class IndexView<geometry::Pose2d>;
  template class IndexView<geometry::Pose2dWithCurvature>;



}
