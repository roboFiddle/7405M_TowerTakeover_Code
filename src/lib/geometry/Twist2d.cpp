//
// Created by alexweiss on 7/2/19.
//

#include "Twist2d.hpp"
#include "../utility/Utility.hpp"
#include <cmath>
#include <sstream>

namespace geometry {
    Twist2d::Twist2d(double dx, double dy, double dtheta) {
        dx_ = dx;
        dy_ = dy;
        dtheta_ = dtheta;
    }
    Twist2d Twist2d::scaled(double scale) {
        return Twist2d(dx_ * scale, dy_ * scale, dtheta_ * scale);
    }
    double Twist2d::norm() {
        if (dy_ == 0.0)
            return fabs(dx_);
        return hypot(dx_, dy_);
    }
    double Twist2d::curvature() {
        if (fabs(dtheta_) < EPSILON && norm() < EPSILON)
            return 0.0;
        return dtheta_ / norm();
    }
    std::string Twist2d::toString() {
        std::ostringstream stringStream;
        stringStream << "Twist2d," << std::to_string(dx_) << "," << std::to_string(dy_);
        stringStream << "," << std::to_string(dtheta_);
        return stringStream.str();
    }
}