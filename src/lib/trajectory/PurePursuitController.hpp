//
// Created by alexweiss on 8/3/19.
//

#ifndef INC_7405M_CODE_SRC_LIB_TRAJECTORY_PUREPURSUITCONTROLLER_HPP_
#define INC_7405M_CODE_SRC_LIB_TRAJECTORY_PUREPURSUITCONTROLLER_HPP_

#include <type_traits>
#include "../geometry/interfaces/State.hpp"
#include "../geometry/interfaces/ITranslation2d.hpp"
#include "../geometry/Twist2d.hpp"
#include "../geometry/Pose2d.hpp"
#include "../geometry/Translation2d.hpp"
#include "IPathFollower.hpp"
#include "TrajectoryIterator.hpp"
#include "DistanceView.hpp"

namespace trajectory {

  template <class S>
  class PurePursuitController : IPathFollower {
    static_assert(std::is_base_of<geometry::ITranslation2d<S>, S>::value, "S must derive from State");
    protected:
      TrajectoryIterator<S> iterator_;
      double sampling_dist_;
      double lookahead_;
      double goal_tolerance_;
      bool done_ = false;
      template <class T> static double getDirection(geometry::Pose2d pose, T point);

    public:
      PurePursuitController(DistanceView<S> path, double sampling_dist, double lookahead, double goal_tolerance);
      geometry::Twist2d steer(geometry::Pose2d current_pose);
      bool isDone();
  };


  template <class S>
  class Arc {
    static_assert(std::is_base_of<geometry::ITranslation2d<S>, S>::value, "T must derive from State");
   protected:
    geometry::Translation2d findCenter(geometry::Pose2d pose, S point);
    double findLength(geometry::Pose2d pose, S point, geometry::Translation2d center, double radius);
   public:
    geometry::Translation2d center;
    double radius;
    double length;
    Arc(geometry::Pose2d pose, S point);
  };
}

#endif //INC_7405M_CODE_SRC_LIB_TRAJECTORY_PUREPURSUITCONTROLLER_HPP_
