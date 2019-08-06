//
// Created by alexweiss on 8/3/19.
//

#include <cmath>
#include "TrajectoryUtil.hpp"
#include "../geometry/interfaces/IPose2d.hpp"
#include "../spline/SplineGenerator.hpp"
#include "../spline/QuinticHermiteSpline.hpp"
#include "../utility/Utility.hpp"

namespace trajectory {

  template <class S>
  Trajectory<S> TrajectoryUtil::mirror(Trajectory<S> trajectory) {
    static_assert(std::is_base_of<geometry::IPose2d<S>, S>::value, "S must derive from State");
    std::vector<S> waypoints;
    for (int i = 0; i < trajectory.size()(); ++i) {
      waypoints.push_back(trajectory.getState(i).mirror());
    }
    return Trajectory<S>(waypoints);
  }

  template <class S>
  Trajectory<TimedState<S>> TrajectoryUtil::mirrorTimed(Trajectory<TimedState<S>> trajectory) {
    std::vector<TimedState<S>> waypoints;
    for (int i = 0; i < trajectory.size()(); ++i) {
      TimedState<S> timed_state = trajectory.getState(i);
      waypoints.push_back(new TimedState<S>(timed_state.state().mirror(), timed_state.t(), timed_state.velocity(), timed_state.acceleration()));
    }
    return Trajectory<S>(waypoints);
  }

  template <class S>
  Trajectory<S> TrajectoryUtil::transform(Trajectory<S> trajectory, geometry::Pose2d transform) {
    std::vector<TimedState<S>> waypoints;
    for (int i = 0; i < trajectory.size(); ++i) {
      waypoints.push_back(trajectory.getState(i).transformBy(transform));
    }
    return Trajectory<S>(waypoints);
  }

  template <class S>
  Trajectory<S> TrajectoryUtil::resample(TrajectoryView<S> trajectory_view, double interval) {
    if (interval <= EPSILON) {
      return new Trajectory<S>();
    }
    int num_states = (int) std::ceil((trajectory_view.last_interpolant() - trajectory_view.first_interpolant()) / interval);
    std::vector<S> states;

    for (int i = 0; i < num_states; ++i) {
      states.push_back(trajectory_view.sample(i * interval + trajectory_view.first_interpolant()).state());
    }
    return Trajectory<S>(states);
  }


  Trajectory<geometry::Pose2dWithCurvature> TrajectoryUtil::trajectoryFromPathFollower(IPathFollower* path_follower,
      geometry::Pose2dWithCurvature start_state, units::QLength step_size, double dcurvature_limit) {

    std::vector<geometry::Pose2dWithCurvature> samples;
    samples.push_back(start_state);
    geometry::Pose2dWithCurvature current_state = start_state;
    while (!path_follower->isDone()) {
      // Get the desired steering command.
      geometry::Twist2d raw_steering_command = path_follower->steer(current_state.pose());

      // Truncate to the step size.
      geometry::Twist2d steering_command = raw_steering_command;
      if (steering_command.norm() > step_size) {
        steering_command = steering_command.scaled((step_size / steering_command.norm()).getValue());
      }

      // Apply limits on spatial derivative of curvature, if desired.
      double dcurvature = ((steering_command.curvature() - current_state.curvature()) / steering_command.norm()).getValue();
      bool curvature_valid = !std::isnan(dcurvature) && !std::isinf(dcurvature)
          && !std::isnan(current_state.curvature()) && !std::isinf((current_state.curvature()));
      if (dcurvature > dcurvature_limit && curvature_valid) {
        steering_command = geometry::Twist2d(steering_command.dx_, steering_command.dy_,
                                             ((dcurvature_limit * steering_command.norm() + current_state.curvature())
                                           * steering_command.norm()).getValue() * units::radian);
      } else if (dcurvature < -dcurvature_limit && curvature_valid) {
        steering_command = geometry::Twist2d(steering_command.dx_, steering_command.dy_,
                                             ((-dcurvature_limit * steering_command.norm() + current_state.curvature())
                                           * steering_command.norm()).getValue() * units::radian);
      }

      // Calculate the new state.
      // Use the average curvature over the interval to compute the next state.
      geometry::Twist2d average_steering_command = !curvature_valid
                                               ? steering_command
                                               : geometry::Twist2d(steering_command.dx_, steering_command.dy_,
                                                             (current_state.curvature() + 0.5 * dcurvature * steering_command.norm().getValue())
                                                                 * steering_command.norm().getValue());

      current_state = geometry::Pose2dWithCurvature(
          current_state.pose().transformBy(geometry::Pose2d::exp(average_steering_command)),
          steering_command.curvature());

      if (!path_follower->isDone()) {
        samples.push_back(current_state);
      }
    }

    return Trajectory<geometry::Pose2dWithCurvature>(samples);
  }

  Trajectory<geometry::Pose2dWithCurvature> TrajectoryUtil::trajectoryFromSplineWaypoints(std::vector<geometry::Pose2d> waypoints, double maxDx, double maxDy, double maxDTheta) {
    std::vector<spline::QuinticHermiteSpline> splines;
    for (int i = 1; i < waypoints.size(); ++i) {
      splines.push_back(spline::QuinticHermiteSpline(waypoints.at(i - 1), waypoints.at(i)));
    }
    spline::QuinticHermiteSpline::optimizeSpline(&splines);
    return trajectoryFromSplines(splines, maxDx, maxDy, maxDTheta);
  }

  template <class S>
  Trajectory<geometry::Pose2dWithCurvature> TrajectoryUtil::trajectoryFromSplines(std::vector<S> splines, double maxDx, double maxDy, double maxDTheta) {
    std::vector<geometry::Pose2dWithCurvature> x = spline::SplineGenerator::parameterizeSplines(&splines, maxDx, maxDy, maxDTheta);
    return Trajectory<geometry::Pose2dWithCurvature>(x);
  }

}