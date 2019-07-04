//
// Created by alexweiss on 7/3/19.
//

#include "testSpline.hpp"
#include "../lib/geometry/Rotation2d.hpp"
#include "../lib/geometry/Translation2d.hpp"
#include "../lib/geometry/Pose2d.hpp"
#include "../lib/geometry/Twist2d.hpp"
#include "../lib/spline/CubicHermiteSpline.hpp"
#include <cmath>
#include <stdio.h>

namespace test {
    void testSpline::testCubicSpline() {
        geometry::Pose2d start(geometry::Translation2d(0,0), geometry::Rotation2d::fromDegrees(90));
        geometry::Pose2d end(geometry::Translation2d(5,10), geometry::Rotation2d::fromDegrees(0));
        spline::CubicHermiteSpline spline(start, end);

        std::printf("%f %f %f %f \n", spline.ax_, spline.bx_, spline.cx_, spline.dx_);
        std::printf("%f %f %f %f \n", spline.ay_, spline.by_, spline.cy_, spline.dy_);
        std::printf("%f %f \n", spline.getCurvature(0), spline.getVelocity(0));
        std::printf("%f %f \n", spline.getCurvature(1), spline.getVelocity(1));

    }
}