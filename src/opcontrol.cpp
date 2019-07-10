#include "main.h"

#include "tests/testGeometry.hpp"
#include "tests/testPhysics.hpp"
#include "tests/testSpline.hpp"
#include "lib/utility/Units.hpp"
#include "lib/geometry/Rotation2d.hpp"

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

double x(units::QLength y) {
  return y.getValue();
}
void opcontrol() {
  //test::testGeometry::testRotation2d();
  //test::testGeometry::testTranslation2d();
  //test::testGeometry::testPose2d();
  //test::testGeometry::testTwist();
  //test::testPhysics::testDriveCharacterization();
  //test::testPhysics::testDCMotorTransmission();
  test::testPhysics::testDifferentialDrive();
  //test::testSpline::testQuinticSpline();
  //test::testSpline::testSplineGenerator();

}
