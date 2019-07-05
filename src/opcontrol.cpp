#include "main.h"
#include "lib/geometry/Rotation2d.hpp"
#include "lib/geometry/Translation2d.hpp"
#include "lib/geometry/Pose2d.hpp"
#include "lib/geometry/Twist2d.hpp"
#include "lib/spline/CubicHermiteSpline.hpp"
#include "lib/spline/QuinticHermiteSpline.hpp"
#include "lib/spline/SplineGenerator.hpp"


#include <stdio.h>

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    geometry::Pose2d p1(geometry::Translation2d(0, 0), geometry::Rotation2d::fromDegrees(0));
    geometry::Pose2d p2(geometry::Translation2d(1, 3), geometry::Rotation2d::fromDegrees(-45));
    geometry::Pose2d p3(geometry::Translation2d(2, 2), geometry::Rotation2d::fromDegrees(0));
    geometry::Pose2d p4(geometry::Translation2d(3, 5), geometry::Rotation2d::fromDegrees(45));
    geometry::Pose2d p5(geometry::Translation2d(4, 7), geometry::Rotation2d::fromDegrees(90));

    std::vector<spline::QuinticHermiteSpline> splines;
    splines.push_back(spline::QuinticHermiteSpline(p1,p2));
    splines.push_back(spline::QuinticHermiteSpline(p2,p3));
    splines.push_back(spline::QuinticHermiteSpline(p3,p4));
    splines.push_back(spline::QuinticHermiteSpline(p4,p5));

    spline::QuinticHermiteSpline::optimizeSpline(&splines);

    std::vector<geometry::Pose2dWithCurvature> samples = spline::SplineGenerator::parameterizeSplines(&splines);

    for (geometry::Pose2dWithCurvature sample : samples) {
        std::printf("(%f, %f)\n", sample.translation().x(), sample.translation().y());
    }
}
