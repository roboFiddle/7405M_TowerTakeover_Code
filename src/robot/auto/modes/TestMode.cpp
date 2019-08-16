//
// Created by alexweiss on 8/14/19.
//

#include "TestMode.hpp"
#include "../../Constants.hpp"
#include "../../paths/TrajectorySet.hpp"

namespace auton {
  void TestMode::routine() {
    printf("started test mode\n");
    /*runAction(new actions::OpenLoopDriveAction(util::DriveSignal(100.0, 100.0), 1.0));
    runAction(new actions::WaitAction(1.5));
    runAction(new actions::OpenLoopDriveAction(util::DriveSignal(-100.0, -100.0), 1.0));*/

    std::vector<geometry::Pose2d> waypoints;

    waypoints.push_back(geometry::Pose2d(0.0, 0.0, geometry::Rotation2d::fromDegrees(0.0)));
    waypoints.push_back(geometry::Pose2d(.5, .5, geometry::Rotation2d::fromDegrees(60.0)));

    trajectory::Trajectory<geometry::Pose2dWithCurvature>
        traj = trajectory::TrajectoryUtil::trajectoryFromSplineWaypoints(waypoints,
                                                                         0.05,
                                                                         0.02,
                                                                         geometry::Rotation2d::fromDegrees(3.0).getRadians());

    physics::DCMotorTransmission transmission(
        constants::RobotConstants::kDriveSpeedPerVolt,
        constants::RobotConstants::kDriveTorquePerVolt,
        constants::RobotConstants::kDriveFrictionVoltage);
    physics::DifferentialDrive drive_model(
        constants::RobotConstants::kRobotMass,
        constants::RobotConstants::kRobotMoment,
        constants::RobotConstants::kRobotAngularDrag,
        constants::RobotConstants::kDriveWheelRadius,
    constants::RobotConstants::kDriveWheelTrackWidth / 2.0 * constants::RobotConstants::kTrackScrubFactor,
        transmission, transmission
    );

    trajectory::DifferentialDriveDynamicsConstraint<geometry::Pose2dWithCurvature> drive_constraints(&drive_model, 12);
    std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature> *> constraints_list;
    constraints_list.push_back(&drive_constraints);
    // Generate the timed trajectory.

    trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>>
        timed_trajectory = trajectory::TimingUtil::timeParameterizeTrajectory(
        false, trajectory::DistanceView<geometry::Pose2dWithCurvature>(&traj), .05, constraints_list,
        0.0, 0.0, 2.5, 2.5);

    for(int i = 0; i < timed_trajectory.length(); i++) {
      std::cout << timed_trajectory.getState(i).toString() << std::endl;
    }

    std::shared_ptr<trajectory::TimedView<geometry::Pose2dWithCurvature>> ptr_to_timed_view = std::make_shared<trajectory::TimedView<geometry::Pose2dWithCurvature>>(&timed_trajectory);
    trajectory::TrajectoryIterator<trajectory::TimedState<geometry::Pose2dWithCurvature>> it(ptr_to_timed_view);

    for(int i = 0; i < timed_trajectory.length(); i++) {
      std::cout << timed_trajectory.getState(i).toString() << std::endl;
    }

    runAction(new actions::DriveTrajectory(
        it
    ));

    /*trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> timed_trajectory = path_planning::TrajectorySet::instance->get("testForward").get(false);
    std::shared_ptr<trajectory::TimedView<geometry::Pose2dWithCurvature>> ptr_to_timed_view = std::make_shared<trajectory::TimedView<geometry::Pose2dWithCurvature>>(&timed_trajectory);
    trajectory::TrajectoryIterator<trajectory::TimedState<geometry::Pose2dWithCurvature>> it(ptr_to_timed_view);
    runAction(new actions::DriveTrajectory(it));*/
  }

}