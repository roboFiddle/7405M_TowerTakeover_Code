#ifndef CODE_V1_TWIST2D_HPP
#define CODE_V1_TWIST2D_HPP

#include <string>

/**
 * A movement along an arc at constant curvature and velocity.
 */

namespace lib::geometry {
  class Twist2d {
      double dx_, dy_, dtheta_;
      Twist2d(double dx, double dy, double dtheta);
      Twist2d scaled(double scale);
      double norm();
      double curvature();
      std::string toString();
  };
}

#endif //CODE_V1_TWIST2D_HPP
