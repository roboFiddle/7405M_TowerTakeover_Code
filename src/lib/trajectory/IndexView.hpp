//
// Created by alexweiss on 8/4/19.
//

#ifndef INC_7405M_CODE_SRC_LIB_TRAJECTORY_INDEXVIEW_HPP_
#define INC_7405M_CODE_SRC_LIB_TRAJECTORY_INDEXVIEW_HPP_

#include "TrajectorySamplePoint.hpp"

namespace trajectory {
  template<class S>  class Trajectory;
  template <class S> class TrajectoryView;

  template<class T>
  class IndexView : public TrajectoryView<T> {
   public:
    Trajectory<T>* pointer_to_;
    IndexView(Trajectory<T>* x);
    TrajectorySamplePoint<T> sample(double index);
    double last_interpolant();
    double first_interpolant();
    Trajectory<T> trajectory();
  };
}

#endif //INC_7405M_CODE_SRC_LIB_TRAJECTORY_INDEXVIEW_HPP_
