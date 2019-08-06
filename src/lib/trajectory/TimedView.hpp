//
// Created by alexweiss on 8/3/19.
//

#ifndef INC_7405M_CODE_SRC_LIB_TRAJECTORY_TIMEDVIEW_HPP_
#define INC_7405M_CODE_SRC_LIB_TRAJECTORY_TIMEDVIEW_HPP_

#include <type_traits>
#include "../geometry/interfaces/State.hpp"
#include "timing/TimedState.hpp"
#include "Trajectory.hpp"
#include "TrajectoryView.hpp"
#include "../geometry/Translation2d.hpp"
#include "../geometry/Rotation2d.hpp"
#include "../geometry/Pose2d.hpp"
#include "../geometry/Pose2dWithCurvature.hpp"

namespace trajectory {

  template <class S>
  class TimedView : TrajectoryView<TimedState<S>> {
    static_assert(std::is_base_of<geometry::State<S>, S>::value, "S must derive from State");
    protected:
      Trajectory<TimedState<S>> trajectory_;
      double start_t_;
      double end_t_;
    public:
      TimedView(Trajectory<TimedState<S>> trajectory);
      double first_interpolant();
      double last_interpolant();
      TrajectorySamplePoint<TimedState<S>> sample(double t);
      Trajectory<TimedState<S>> trajectory();

  };

}

#endif //INC_7405M_CODE_SRC_LIB_TRAJECTORY_TIMEDVIEW_HPP_
