#include "main.h"

#include "tests/testGeometry.hpp"


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

    /* std::vector<util::Point> points;
    points.push_back(util::Point(0,1));
    points.push_back(util::Point(1,3));
    points.push_back(util::Point(3,13));
    points.push_back(util::Point(5,31));

    util::PolynomialRegression test(points, 2);

    std::printf("%d \n", test.coeffs_.size());

    for (int i = 0; i < 3; i++) {
        std::printf("(%d, %f)\n", i, test.beta(i));
    }
  std::printf("(%d, %f)\n", 12, test.predict(12)); */
    test::testGeometry::testRotation2d();
    test::testGeometry::testTranslation2d();
    test::testGeometry::testPose2d();
    test::testGeometry::testTwist();
}
