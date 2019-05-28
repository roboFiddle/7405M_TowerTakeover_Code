#ifndef CODE_V1_POSE2DWITHCURVATURE_HPP
#define CODE_V1_POSE2DWITHCURVATURE_HPP

#include "interfaces/IPose2d.hpp"
#include "interfaces/ICurvature.hpp"
#include "Pose2d.hpp"
#include "Rotation2d.hpp"
#include "Translation2d.hpp"
#include <string>

namespace wlib::geometry {
  class Pose2dWithCurvature : IPose2d<Pose2dWithCurvature>, ICurvature<Pose2dWithCurvature> {
    private:
      Pose2d pose_;
      double curvature_;
      double dcurvature_ds_;
    public:
      Pose2dWithCurvature();
      Pose2dWithCurvature(final Pose2d pose, double curvature);
      Pose2dWithCurvature(final Pose2d pose, double curvature, double dcurvature_ds);
      Pose2dWithCurvature(final Translation2d translation, final Rotation2d rotation, double curvature);
      Pose2dWithCurvature(final Translation2d translation, final Rotation2d rotation, double curvature, double dcurvature_ds);
      Pose2d getPose();
      Pose2dWithCurvature transformBy(Pose2d transform);
      Pose2dWithCurvature mirror();
      double getCurvature();
      double getDCurvatureDs();
      Translation2d getTranslation();
      Rotation2d getRotation();
      Pose2dWithCurvature interpolate(Pose2dWithCurvature other, double x);
      double distance(Pose2dWithCurvature other);
      bool operator==(Pose2dWithCurvature other);
      std::string toCSV();
      std::string toString();
  };
}

#endif //CODE_V1_POSE2DWITHCURVATURE_HPP
