#ifndef CODE_V1_POSE2D_HPP
#define CODE_V1_POSE2D_HPP

#include "Translation2d.hpp"
#include "Rotation2d.hpp"
#include "Twist2d.hpp"
#include <string>

namespace wlib::geometry {
  class Pose2d : IPose2d<Pose2d> {
    protected:
      Translation2d translation_;
      Rotation2d rotation_;
    public:
      Pose2d();
      Pose2d(double x, double y, Rotation2d rotation);
      Pose2d(Translation2d translation, Rotation2d rotation);
      Translation2d translation();
      Rotation2d rotation()
      Pose2d transformBy(Pose2d other);
      Pose2d inverse();
      Pose2d normal();
      // Find intersection point based on headings
      Translation2d intersection(Pose2d other);
      bool IsLinear(Pose2d other);
      // Interpolation based on CONSTANT CURVATURE
      Pose2d interpolate(Pose2d other, double x);
      Pose2d mirror();
      double distance(Pose2d other);
      std::string toCSV();
      std::string toString();
      bool operator==(Pose2d other);

      static Pose2d fromTranslation(final Translation2d translation);
      static Pose2d fromRotation(final Rotation2d rotation);
      // Resulting Pose after a Twist
      static Pose2d exp(final Twist2d delta);
      // Given final Pose, find Twist
      static Twist2d log(final Pose2d transform);
      static Translation2d intersectionInternal(Pose2d a, Pose2d b);



  };
}

#endif //CODE_V1_POSE2D_HPP
