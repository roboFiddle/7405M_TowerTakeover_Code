//
// Created by alexweiss on 8/3/19.
//

#ifndef INC_7405M_CODE_SRC_LIB_TRAJECTORY_TRAJECTORYUTIL_HPP_
#define INC_7405M_CODE_SRC_LIB_TRAJECTORY_TRAJECTORYUTIL_HPP_

#include "../geometry/interfaces/IPose2d.hpp"
#include "../geometry/Pose2dWithCurvature.hpp"
#include "../geometry/Pose2d.hpp"
#include "IPathFollower.hpp"
#include "Trajectory.hpp"
#include "TrajectoryView.hpp"
#include "timing/TimedState.hpp"



namespace trajectory {
  class TrajectoryUtil {
    public:
      template <class S>
      static Trajectory<S> mirror(Trajectory<S> trajectory);

      template <class S>
      static Trajectory<TimedState<S>> mirrorTimed(Trajectory<TimedState<S>> trajectory);

      template <class S>
      static Trajectory<S> transform(Trajectory<S> trajectory, geometry::Pose2d transform);

      template <class S>
      static Trajectory<S> resample(TrajectoryView<S> trajectory_view, double interval);


      static Trajectory<geometry::Pose2dWithCurvature> trajectoryFromPathFollower(IPathFollower path_follower,
          geometry::Pose2dWithCurvature start_state, double step_size, double dcurvature_limit);

      static Trajectory<geometry::Pose2dWithCurvature> trajectoryFromSplineWaypoints(
          std::list<geometry::Pose2d> waypoints, double maxDx, double maxDy, double maxDTheta);

      template <class S>
      static Trajectory<geometry::Pose2dWithCurvature> trajectoryFromSplines(std::list<S> splines, double maxDx, double maxDy, double maxDTheta);
  };
}

#endif //INC_7405M_CODE_SRC_LIB_TRAJECTORY_TRAJECTORYUTIL_HPP_
