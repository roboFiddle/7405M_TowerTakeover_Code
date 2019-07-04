#ifndef CODE_V1_POSE2DWITHCURVATURE_HPP
#define CODE_V1_POSE2DWITHCURVATURE_HPP

#include "Pose2d.hpp"
#include "Rotation2d.hpp"
#include "Translation2d.hpp"
#include <string>

namespace geometry {
  class Pose2dWithCurvature {
    private:
      Pose2d pose_;
      double curvature_;
      double dcurvature_ds_;
    public:
      Pose2dWithCurvature();
      Pose2dWithCurvature(Pose2d pose, double curvature);
      Pose2dWithCurvature(Pose2d pose, double curvature, double dcurvature_ds);
      Pose2dWithCurvature(Translation2d translation, Rotation2d rotation, double curvature);
      Pose2dWithCurvature(Translation2d translation, Rotation2d rotation, double curvature, double dcurvature_ds);
      Pose2d pose();
      Pose2dWithCurvature transformBy(Pose2d transform);
      Pose2dWithCurvature mirror();
      double curvature();
      double dcurvature();
      Translation2d translation();
      Rotation2d rotation();
      Pose2dWithCurvature interpolate(Pose2dWithCurvature other, double x);
      double distance(Pose2dWithCurvature other);
      bool operator==(Pose2dWithCurvature other);
      Pose2dWithCurvature& operator=(Pose2dWithCurvature other);
      std::string toCSV();
      std::string toString();
  };
}

#endif //CODE_V1_POSE2DWITHCURVATURE_HPP
