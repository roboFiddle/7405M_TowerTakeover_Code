//
// Created by alexweiss on 8/4/19.
//

#include "IndexView.hpp"

namespace trajectory {

  template<class T>
  IndexView<T>::IndexView(Trajectory<T>* x) {
    pointer_to_ = x; // TODO: try removing reference
  }

  template<class T>
  TrajectorySamplePoint<T> IndexView<T>::sample(double index) {
    return pointer_to_->getInterpolated(index);
  }

  template<class T>
  double IndexView<T>::last_interpolant() {
    return MAX(0.0, pointer_to_->length() - 1);
  }

  template<class T>
  double IndexView<T>::first_interpolant() {
    return 0.0;
  }

  template<class T>
  Trajectory<T> IndexView<T>::trajectory() {
    return *pointer_to_;
  }

}
