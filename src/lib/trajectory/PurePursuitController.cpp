  //
// Created by alexweiss on 8/3/19.
//

#include "PurePursuitController.hpp"
#include "../utility/Utility.hpp"

namespace trajectory {

  template <class S>
  PurePursuitController<S>::PurePursuitController(DistanceView<S> path, double sampling_dist, double lookahead, double goal_tolerance) {
    sampling_dist_ = sampling_dist;
    lookahead_ = lookahead;
    goal_tolerance_ = goal_tolerance;
    iterator_ = TrajectoryIterator<S>(path);
  }

  template <class S>
  geometry::Twist2d PurePursuitController<S>::steer(geometry::Pose2d current_pose) {
    done_ = done_ || (iterator_.isDone()
        && current_pose.translation().distance(iterator_.getState().getTranslation()) <= goal_tolerance_);
    if (isDone()) {
      return geometry::Twist2d(0.0, 0.0, 0.0);
    }

    double remaining_progress = iterator_.getRemainingProgress();
    double goal_progress = 0.0;
    // Find the first point > lookahead distance away from current_pose, or the last point otherwise.
    for (double progress = 0.0; progress <= remaining_progress; progress = MIN(remaining_progress,
                                                                                    progress + sampling_dist_)) {
      double dist = current_pose.translation().distance(iterator_.preview(progress).state().getTranslation());
      if (dist > lookahead_) {
        if (goal_progress == 0.0 && !iterator_.isDone()) {
          // Make sure we don't get stuck due to numerical issues when sampling dist is large relative to
          // lookahead.
          goal_progress = progress;
        }
        break;
      }
      goal_progress = progress;
      if (progress == remaining_progress) {
        break;
      }
    }
    iterator_.advance(goal_progress);
    Arc<S> arc = new Arc<S>(current_pose, iterator_.getState());
    if (arc.length < EPSILON) {
      return geometry::Twist2d(0.0, 0.0, 0.0);
    }
    else {
      return geometry::Twist2d(arc.length, 0.0, arc.length / arc.radius);
    }

  }

  template <class S>
  bool PurePursuitController<S>::isDone() {
    return done_;
  }

  template <class S> template <class T>
  double PurePursuitController<S>::getDirection(geometry::Pose2d pose, T point) {
    geometry::Translation2d poseToPoint = Translation2d(pose.translation(), point.getTranslation());
    geometry::Translation2d robot = pose.rotation().toTranslation();
    double cross = (robot.x() * poseToPoint.y() - robot.y() * poseToPoint.x()).getValue();
    return (cross < 0.) ? -1. : 1.; // if robot < pose turn left
  }

  template <class S>
  Arc<S>::Arc(geometry::Pose2d pose, S point) {
    center = findCenter(pose, point);
    radius = Translation2d(center, point.getTranslation()).norm();
    length = findLength(pose, point, center, radius);
    radius *= getDirection(pose, point);
  }

  template <class S>
  geometry::Translation2d Arc<S>::findCenter(geometry::Pose2d pose, S point) {
    geometry::Translation2d poseToPointHalfway = pose.translation().interpolate(point.getTranslation(), 0.5);
    geometry::Rotation2d normal = pose.translation().inverse().translateBy(poseToPointHalfway).direction()
        .normal();
    geometry::Pose2d perpendicularBisector = geometry::Pose2d(poseToPointHalfway, normal);
    geometry::Pose2d normalFromPose = geometry::Pose2d(pose.translation(),
                                             pose.rotation().normal());
    if (normalFromPose.IsLinear(perpendicularBisector.normal())) {
      // Special case: center is poseToPointHalfway.
      return poseToPointHalfway;
    }
    return normalFromPose.intersection(perpendicularBisector);
  }

  template <class S>
  double Arc<S>::findLength(geometry::Pose2d pose, S point, geometry::Translation2d center, double radius) {
    geometry::Translation2d centerToPoint = geometry::Translation2d(center, point.getTranslation());
    geometry::Translation2d centerToPose = geometry::Translation2d(center, pose.translation());
    // If the point is behind pose, we want the opposite of this angle. To determine if the point is behind,
    // check the sign of the cross-product between the normal vector and the vector from pose to point.
    bool behind = util::sgn(
        geometry::Translation2d::cross(pose.rotation().normal().toTranslation(),
                            Translation2d(pose.translation(), point.getTranslation()))) > 0.0;
    geometry::Rotation2d angle = geometry::Translation2d::getAngle(centerToPose, centerToPoint);
    return radius * (behind ? 2.0 * M_PI - std::fabs(angle.getRadians()) : std::fabs(angle.getRadians()));
  }
}