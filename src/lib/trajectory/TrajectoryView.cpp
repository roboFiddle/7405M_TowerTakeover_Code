//
// Created by alexweiss on 8/5/19.
//

#include "TrajectoryView.hpp"

namespace trajectory {
  template class TrajectoryView<geometry::Translation2d>;
  template class TrajectoryView<geometry::Rotation2d>;
  template class TrajectoryView<geometry::Pose2d>;
  template class TrajectoryView<geometry::Pose2dWithCurvature>;
}