//
// Created by alexweiss on 8/3/19.
//

#ifndef INC_7405M_CODE_SRC_LIB_TRAJECTORY_TRAJECTORYITERATOR_HPP_
#define INC_7405M_CODE_SRC_LIB_TRAJECTORY_TRAJECTORYITERATOR_HPP_

#include <type_traits>
#include <memory>
#include "../geometry/interfaces/State.hpp"
#include "Trajectory.hpp"
#include "TrajectoryView.hpp"
#include "TrajectorySamplePoint.hpp"
#include "../geometry/Translation2d.hpp"
#include "../geometry/Rotation2d.hpp"
#include "../geometry/Pose2d.hpp"
#include "../geometry/Pose2dWithCurvature.hpp"


namespace trajectory {

  template <class S>
  class TrajectoryIterator {
    static_assert(std::is_base_of<geometry::State<S>, S>::value, "S must derive from State");
    protected:
      std::shared_ptr<TrajectoryView<S>> view_;
      double progress_ = 0.0;
      TrajectorySamplePoint<S> current_sample_;

    public:
      TrajectoryIterator(std::shared_ptr<TrajectoryView<S>> view);
      void setup();
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
