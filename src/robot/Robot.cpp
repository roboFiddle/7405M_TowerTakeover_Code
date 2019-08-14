//
// Created by alexweiss on 8/9/19.
//

#include "../lib/meecan_lib.hpp"
#include "../tests/testsInclude.hpp"
#include "main.h"
#include "Robot.hpp"
#include "Constants.hpp"
#include "loops/Loop.hpp"
#include "loops/Looper.hpp"

namespace meecan {
  Robot::RobotManager instance;

  Robot::Robot() {
    printf("robot construct\n");
    setupMainLoop();
  }

  void Robot::robotInit() {
    pros::lcd::initialize();
    pros::lcd::print(2, "robot init");
    mainLooper->enable();
  }
  void Robot::disabledInit() {
    pros::lcd::print(1, "disabled init %d", pros::millis());
  }
  void Robot::disabledLoop() {
    pros::lcd::print(0, "disabled loop %d", pros::millis());
  }
  void Robot::autonomousInit() {
    pros::lcd::print(1, "auton init %d", pros::millis());
  }
  void Robot::autonomousLoop() {
    pros::lcd::print(0, "auton loop %d", pros::millis());
  }
  void Robot::driverInit() {
    pros::lcd::print(1, "driver init %d", pros::millis());
    //test::testTrajectory::realisticConstants();

    std::vector<geometry::Pose2d> waypoints;
    geometry::Pose2d a(0.0, 0.0, geometry::Rotation2d::fromDegrees(90.0));
    geometry::Pose2d b(0.5, 0.25, geometry::Rotation2d::fromDegrees(0));
    geometry::Pose2d c(1.0, 0.0, geometry::Rotation2d::fromDegrees(270.0));

    waypoints.push_back(a);
    waypoints.push_back(b);
    waypoints.push_back(c);

    /*std::vector<spline::QuinticHermiteSpline> splines;
    splines.push_back(spline::QuinticHermiteSpline(a, b));
    splines.push_back(spline::QuinticHermiteSpline(b, c));

    spline::QuinticHermiteSpline::optimizeSpline(&splines);
    std::vector<geometry::Pose2dWithCurvature> samples = spline::SplineGenerator::parameterizeSplines(&splines);*/

    trajectory::Trajectory<geometry::Pose2dWithCurvature> test = trajectory::TrajectoryUtil::trajectoryFromSplineWaypoints(waypoints, 0.05, 0.01, 0.1);

    units::QMass kRobotMassKg = 8;
    units::QMoment kRobotAngularInertia = 12;
    units::QAngularDrag kRobotAngularDrag = 0.0;
    units::QLength kWheelRadius = (2.0 * units::inch);
    units::QLength kWheelbaseRadius =  (18.0 / 2.0) * units::inch;
    printf("WHEEL RADIUS %f\n", kWheelRadius);
    printf("WHEELBASE RADIUS %f\n", kWheelbaseRadius);
    physics::DCMotorTransmission
        transmission(1.903, .175, 1);
    physics::DifferentialDrive drive(kRobotMassKg,
                                     kRobotAngularInertia,
                                     kRobotAngularDrag,
                                     kWheelRadius,
                                     kWheelbaseRadius,
                                     transmission,
                                     transmission);
    // Create the constraint that the robot must be able to traverse the trajectory without ever applying more
    // than 10V.
    trajectory::DifferentialDriveDynamicsConstraint<geometry::Pose2dWithCurvature> drive_constraints(&drive, 12);
    std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature> *> constraints_list;
    constraints_list.push_back(&drive_constraints);
    // Generate the timed trajectory.

    trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>>
        timed_trajectory = trajectory::TimingUtil::timeParameterizeTrajectory(
        false, trajectory::DistanceView<geometry::Pose2dWithCurvature>(&test), .05, constraints_list,
        0.0, 0.0, 12.0 * 14.0, 12.0 * 10.0);


    /* for(int i = 0; i < timed_trajectory.length(); i++) {
      printf("(%f, %f)\n", timed_trajectory.getState(i).state().translation().x(), timed_trajectory.getState(i).state().translation().y());
    } */

    double kDt = 0.05;
    std::shared_ptr<trajectory::TimedView<geometry::Pose2dWithCurvature>> ptr_to_timed_view = std::make_shared<trajectory::TimedView<geometry::Pose2dWithCurvature>>(&timed_trajectory);
    trajectory::TrajectoryIterator<trajectory::TimedState<geometry::Pose2dWithCurvature>> it(ptr_to_timed_view);
    while (!it.isDone()) {
      trajectory::TrajectorySamplePoint<trajectory::TimedState<geometry::Pose2dWithCurvature>>
          sample(trajectory::TimedState<geometry::Pose2dWithCurvature>(), 0, 0);
      sample = it.advance(kDt);
      trajectory::TimedState<geometry::Pose2dWithCurvature> state = sample.state();

      physics::DifferentialDrive::DriveDynamics dynamics = drive.solveInverseDynamics(
          physics::DifferentialDrive::ChassisState<units::QSpeed, units::QAngularSpeed>(state.velocity(), state.velocity() * state.state().curvature()),
          physics::DifferentialDrive::ChassisState<units::QAcceleration, units::QAngularAcceleration>((state.acceleration()), state.acceleration() * state.state().curvature()));

      //printf("(%f, %f)\n", state.t(), dynamics.wheel_velocity.left_);
      printf("(%f, %f)\n", state.state().translation().x(),state.state().translation().y());

    }
  }
  void Robot::driverLoop() {
    pros::lcd::print(0, "driver loop %d", pros::millis());
  }
}