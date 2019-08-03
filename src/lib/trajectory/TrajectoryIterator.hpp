//
// Created by alexweiss on 8/3/19.
//

#ifndef INC_7405M_CODE_SRC_LIB_TRAJECTORY_TRAJECTORYITERATOR_HPP_
#define INC_7405M_CODE_SRC_LIB_TRAJECTORY_TRAJECTORYITERATOR_HPP_

#include <type_traits>
#include "../geometry/interfaces/State.hpp"
#include "TrajectoryView.hpp"
#include "TrajectorySamplePoint.hpp"


namespace trajectory {

  template <class S>
  class TrajectoryIterator {
    static_assert(std::is_base_of<geometry::State<S>, S>::value, "S must derive from State");
    protected:
      TrajectoryView<S> view_;
      double progress_ = 0.0;
      TrajectorySamplePoint<S> current_sample_;

    public:
      TrajectoryIterator(TrajectoryView<S> view);
      bool isDone();
      double getProgress();
      double getRemainingProgress();
      TrajectorySamplePoint<S> getSample();
      S getState();
      TrajectorySamplePoint<S> advance(double additional_progress);
      TrajectorySamplePoint<S> preview(double additional_progress);
      Trajectory<S> trajectory();



  };
}

#endif //INC_7405M_CODE_SRC_LIB_TRAJECTORY_TRAJECTORYITERATOR_HPP_
