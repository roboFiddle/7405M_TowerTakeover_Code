//
// Created by alexweiss on 8/14/19.
//

#include "TrajectorySet.hpp"
#include "DriveMotionPlanner.hpp"
#include "PathConstants.hpp"
#include "../../lib/trajectory/timing/VelocityLimitRegionConstraint.hpp"

namespace path_planning {
  TrajectorySet::TrajectorySetManager TrajectorySet::instance;

  MirroredTrajectory::MirroredTrajectory(trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> forward) {
    forward_ = forward;
    backward_ = trajectory::TrajectoryUtil::mirrorTimed(forward);
  }
  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> MirroredTrajectory::get(bool back) {
    if(back)
      return backward_;
    return forward_;
  }

  TrajectorySet::TrajectorySet() {
    complete_ = false;
  }
  bool TrajectorySet::isDoneGenerating() {
    return complete_;
  }
  void TrajectorySet::addToMap(std::string name, MirroredTrajectory traj) {
    trajectories_.insert({name, traj});
  }
  bool TrajectorySet::inSet(std::string name) {
    return trajectories_.count(name);
  }
  MirroredTrajectory TrajectorySet::get(std::string name) {
    return trajectories_.at(name);
  }
  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::backJForward() {
    std::vector<geometry::Pose2d> waypoints;
    waypoints.push_back(geometry::Pose2d(12 * units::inch, 50.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));
    waypoints.push_back(geometry::Pose2d(36 * units::inch, 50.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));

    geometry::Translation2d A(32 * units::inch, 46 * units::inch);
    geometry::Translation2d B(53 * units::inch, 53 * units::inch);
    std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> constraints;

    trajectory::VelocityLimitRegionConstraint<geometry::Pose2dWithCurvature> x(A, B, constants::PathConstants::kMaxVelocity * 0.15);
    //constraints.push_back(&x);

    return DriveMotionPlanner::generateTrajectory(false, waypoints, constraints,
                                                  constants::PathConstants::kMaxVelocity, constants::PathConstants::kMaxAccel, 8.0);
  }
  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::backToLine() {
    std::vector<geometry::Pose2d> waypoints;
    waypoints.push_back(geometry::Pose2d(0 * units::inch, 0 * units::inch, geometry::Rotation2d::fromDegrees(0)));
    //waypoints.push_back(geometry::Pose2d(43 * units::inch, 36 * units::inch, geometry::Rotation2d::fromDegrees(0)));
    waypoints.push_back(geometry::Pose2d(30 * units::inch, 0 * units::inch, geometry::Rotation2d::fromDegrees(0)));


    std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> noConstraints;

    return DriveMotionPlanner::generateTrajectory(false, waypoints, noConstraints,
                                                  constants::PathConstants::kMaxVelocity, constants::PathConstants::kMaxAccel, 8.0);
  }
  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::backLineForward() {
    std::vector<geometry::Pose2d> waypoints;
    waypoints.push_back(geometry::Pose2d(9 * units::inch, 26.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));
    //waypoints.push_back(geometry::Pose2d(50 * units::inch, 26.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));
    waypoints.push_back(geometry::Pose2d(59.5 * units::inch, 26.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));

    geometry::Translation2d A1(9 * units::inch, 23 * units::inch);
    geometry::Translation2d B1(35 * units::inch, 32 * units::inch);
    trajectory::VelocityLimitRegionConstraint<geometry::Pose2dWithCurvature> x1(A1, B1, constants::PathConstants::kMaxVelocity * 0.4);

    geometry::Translation2d A3(35 * units::inch, 23 * units::inch);
    geometry::Translation2d B3(56 * units::inch, 32 * units::inch);
    trajectory::VelocityLimitRegionConstraint<geometry::Pose2dWithCurvature> x3(A3, B3, constants::PathConstants::kMaxVelocity * 0.325);

    std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> constraints;
    constraints.push_back(&x1);
    constraints.push_back(&x3);

    return DriveMotionPlanner::generateTrajectory(false, waypoints, constraints,
                                                  constants::PathConstants::kMaxVelocity * 0.5, constants::PathConstants::kMaxAccel, 8.0);
  }

  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::backBackForTowerCube() {
    std::vector<geometry::Pose2d> waypoints;
    waypoints.push_back(geometry::Pose2d(45 * units::inch, 26.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));
    waypoints.push_back(geometry::Pose2d(51.5 * units::inch, 26.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));

    std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> noConstraints;

    return DriveMotionPlanner::generateTrajectory(false, waypoints, noConstraints,
                                                  constants::PathConstants::kMaxVelocity * 1.5, constants::PathConstants::kMaxAccel * 0.5, 8.0);
  }

  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::backGetTowerCube() {
    std::vector<geometry::Pose2d> waypoints;
    waypoints.push_back(geometry::Pose2d(40 * units::inch, 26.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));
    waypoints.push_back(geometry::Pose2d(45 * units::inch, 26.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));

    std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> noConstraints;

    return DriveMotionPlanner::generateTrajectory(false, waypoints, noConstraints,
                                                  constants::PathConstants::kMaxVelocity * 0.1, constants::PathConstants::kMaxAccel, 8.0);
  }
  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::backGetSecondCube() {
    std::vector<geometry::Pose2d> waypoints;
    waypoints.push_back(geometry::Pose2d(40 * units::inch, 26.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));
    waypoints.push_back(geometry::Pose2d(51 * units::inch, 26.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));

    std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> noConstraints;

    return DriveMotionPlanner::generateTrajectory(false, waypoints, noConstraints,
                                                  constants::PathConstants::kMaxVelocity * 1.5, constants::PathConstants::kMaxAccel, 8.0);
  }

  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::backOffTower() {
    std::vector<geometry::Pose2d> waypoints;
    waypoints.push_back(geometry::Pose2d(40 * units::inch, 26.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));
    waypoints.push_back(geometry::Pose2d(48 * units::inch, 26.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));

    std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> noConstraints;

    return DriveMotionPlanner::generateTrajectory(false, waypoints, noConstraints,
                                                  constants::PathConstants::kMaxVelocity * 0.25, constants::PathConstants::kMaxAccel * 0.5, 8.0);
  }

  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::backAlign(bool l) {
    std::vector<geometry::Pose2d> waypoints;
    waypoints.push_back(geometry::Pose2d(9 * units::inch, 24 * units::inch, geometry::Rotation2d::fromDegrees(0)));
    waypoints.push_back(geometry::Pose2d((l ? 44 : 41) * units::inch, 24 * units::inch, geometry::Rotation2d::fromDegrees(0)));

    std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> noConstraints;

    return DriveMotionPlanner::generateTrajectory(false, waypoints, noConstraints,
                                                  constants::PathConstants::kMaxVelocity * 1.25, constants::PathConstants::kMaxAccel, 8.0);
  }
  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::backSetup() {
    std::vector<geometry::Pose2d> waypoints;
    waypoints.push_back(geometry::Pose2d(7.5 * units::inch, 18 * units::inch, geometry::Rotation2d::fromDegrees(-90)));
    //waypoints.push_back(geometry::Pose2d(40 * units::inch, 26.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));
    waypoints.push_back(geometry::Pose2d(7.5 * units::inch, 8 * units::inch, geometry::Rotation2d::fromDegrees(-90)));

    std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> noConstraints;

    return DriveMotionPlanner::generateTrajectory(false, waypoints, noConstraints,
                                                  constants::PathConstants::kMaxVelocity, constants::PathConstants::kMaxAccel, 8.0);
  }

  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::stackPullBack(bool skills) {
    std::vector<geometry::Pose2d> waypoints;
    waypoints.push_back(geometry::Pose2d(0, 0, geometry::Rotation2d::fromDegrees(0)));
    waypoints.push_back(geometry::Pose2d((skills ? 10 : 15) * units::inch, 0, geometry::Rotation2d::fromDegrees(0)));

    std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> noConstraints;

    return trajectory::TimingUtil::reverseTimed(DriveMotionPlanner::generateTrajectory(false, waypoints, noConstraints,
                                                                                       constants::PathConstants::kMaxVelocity * 1.5, constants::PathConstants::kMaxAccel * 2, 8.0));
  }

  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::pSkillsIntake() {
    std::vector<geometry::Pose2d> waypoints;
    waypoints.push_back(geometry::Pose2d(9 * units::inch, 26.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));
    //waypoints.push_back(geometry::Pose2d(50 * units::inch, 26.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));
    waypoints.push_back(geometry::Pose2d(110 * units::inch, 26.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));

    std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> noConstraints;

    return DriveMotionPlanner::generateTrajectory(false, waypoints, noConstraints,
                                                  constants::PathConstants::kMaxVelocity * 0.35, constants::PathConstants::kMaxAccel, 8.0);
  }
  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::pSkillsSecondIntake() {
    std::vector<geometry::Pose2d> waypoints;
    waypoints.push_back(geometry::Pose2d(9 * units::inch, 26.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));
    //waypoints.push_back(geometry::Pose2d(50 * units::inch, 26.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));
    waypoints.push_back(geometry::Pose2d(42 * units::inch, 26.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));

    std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> constraints;

    return DriveMotionPlanner::generateTrajectory(false, waypoints, constraints,
                                                  constants::PathConstants::kMaxVelocity * 0.4, constants::PathConstants::kMaxAccel, 8.0);
  }

  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::pSkillsSetup() {
    std::vector<geometry::Pose2d> waypoints;
    waypoints.push_back(geometry::Pose2d(9 * units::inch, 26.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));
    //waypoints.push_back(geometry::Pose2d(50 * units::inch, 26.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));
    waypoints.push_back(geometry::Pose2d(20 * units::inch, 26.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));

    std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> noConstraints;


    return DriveMotionPlanner::generateTrajectory(false, waypoints, noConstraints,
                                                  constants::PathConstants::kMaxVelocity * 0.35, constants::PathConstants::kMaxAccel, 8.0);
  }

  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::longWallBump() {
    std::vector<geometry::Pose2d> waypoints;
    waypoints.push_back(geometry::Pose2d(9 * units::inch, 26.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));
    //waypoints.push_back(geometry::Pose2d(50 * units::inch, 26.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));
    waypoints.push_back(geometry::Pose2d(32 * units::inch, 26.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));

    std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> noConstraints;


    return DriveMotionPlanner::generateTrajectory(false, waypoints, noConstraints,
                                                  constants::PathConstants::kMaxVelocity * 0.65, constants::PathConstants::kMaxAccel, 8.0);
  }

  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::alignWithFirstTower() {
    std::vector<geometry::Pose2d> waypoints;
    waypoints.push_back(geometry::Pose2d(0 * units::inch, 26.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));
    //waypoints.push_back(geometry::Pose2d(50 * units::inch, 26.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));
    waypoints.push_back(geometry::Pose2d(14 * units::inch, 26.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));

    std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> noConstraints;


    return DriveMotionPlanner::generateTrajectory(false, waypoints, noConstraints,
                                                  constants::PathConstants::kMaxVelocity * 0.5, constants::PathConstants::kMaxAccel, 8.0);
  }
  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::intakeFirstTower() {
    std::vector<geometry::Pose2d> waypoints;
    waypoints.push_back(geometry::Pose2d(0 * units::inch, 26.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));
    //waypoints.push_back(geometry::Pose2d(50 * units::inch, 26.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));
    waypoints.push_back(geometry::Pose2d(35 * units::inch, 26.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));

    std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> constraints;

    return DriveMotionPlanner::generateTrajectory(false, waypoints, constraints,
                                                  constants::PathConstants::kMaxVelocity * 0.5, constants::PathConstants::kMaxAccel, 8.0);
  }
trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::shortTower(bool l, bool m) {
  std::vector<geometry::Pose2d> waypoints;
  waypoints.push_back(geometry::Pose2d(0 * units::inch, 26.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));
  //waypoints.push_back(geometry::Pose2d(50 * units::inch, 26.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));
  waypoints.push_back(geometry::Pose2d((l ? 12 : (m ? 9 : 6)) * units::inch, 26.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));

  std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> noConstraints;


  return DriveMotionPlanner::generateTrajectory(false, waypoints, noConstraints,
                                                constants::PathConstants::kMaxVelocity * 0.5, constants::PathConstants::kMaxAccel, 8.0);
}
trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::backCurve() {
  std::vector<geometry::Pose2d> waypoints;
  waypoints.push_back(geometry::Pose2d(0 * units::inch, 0 * units::inch, geometry::Rotation2d::fromDegrees(0)));
  //waypoints.push_back(geometry::Pose2d(50 * units::inch, 26.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));
  waypoints.push_back(geometry::Pose2d(8 * units::inch, 0 * units::inch, geometry::Rotation2d::fromDegrees(0)));

  std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> noConstraints;


  return DriveMotionPlanner::generateTrajectory(false, waypoints, noConstraints,
                                                constants::PathConstants::kMaxVelocity * 0.25, constants::PathConstants::kMaxAccel * 0.5, 8.0);
}
trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::alignInThirdTower() {
  std::vector<geometry::Pose2d> waypoints;
  waypoints.push_back(geometry::Pose2d(0 * units::inch, 26.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));
  //waypoints.push_back(geometry::Pose2d(50 * units::inch, 26.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));
  waypoints.push_back(geometry::Pose2d(24 * units::inch, 26.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));

  std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> noConstraints;


  return DriveMotionPlanner::generateTrajectory(false, waypoints, noConstraints,
                                                constants::PathConstants::kMaxVelocity * 0.5, constants::PathConstants::kMaxAccel, 8.0);
}
trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::placeInThirdTower() {
  std::vector<geometry::Pose2d> waypoints;
  waypoints.push_back(geometry::Pose2d(0 * units::inch, 26.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));
  //waypoints.push_back(geometry::Pose2d(50 * units::inch, 26.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));
  waypoints.push_back(geometry::Pose2d(33 * units::inch, 26.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));

  std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> noConstraints;


  return DriveMotionPlanner::generateTrajectory(false, waypoints, noConstraints,
                                                constants::PathConstants::kMaxVelocity * 0.5, constants::PathConstants::kMaxAccel, 8.0);
}

trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::getFrontIntake() {
  std::vector<geometry::Pose2d> waypoints;
  waypoints.push_back(geometry::Pose2d(0 * units::inch, 26.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));
  //waypoints.push_back(geometry::Pose2d(50 * units::inch, 26.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));
  waypoints.push_back(geometry::Pose2d(24 * units::inch, 26.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));

  std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> noConstraints;


  return DriveMotionPlanner::generateTrajectory(false, waypoints, noConstraints,
                                                constants::PathConstants::kMaxVelocity * 0.25, constants::PathConstants::kMaxAccel, 8.0);
}

trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::getFrontSetup() {
  std::vector<geometry::Pose2d> waypoints;
  waypoints.push_back(geometry::Pose2d(0 * units::inch, 26.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));
  //waypoints.push_back(geometry::Pose2d(50 * units::inch, 26.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));
  waypoints.push_back(geometry::Pose2d(10 * units::inch, 26.4 * units::inch, geometry::Rotation2d::fromDegrees(0)));

  std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> noConstraints;


  return DriveMotionPlanner::generateTrajectory(false, waypoints, noConstraints,
                                                constants::PathConstants::kMaxVelocity * 0.5, constants::PathConstants::kMaxAccel, 8.0);
}



  void TrajectorySet::generatorCalls() {
    printf("start\n");
    addToMap("backJForward", MirroredTrajectory(backJForward()));
    addToMap("backToLine", MirroredTrajectory(backToLine()));
    addToMap("backLineForward", MirroredTrajectory(backLineForward()));
    addToMap("backBackForTowerCube", MirroredTrajectory(backBackForTowerCube()));
    addToMap("backOffTower", MirroredTrajectory(backOffTower()));
    addToMap("backGetTowerCube", MirroredTrajectory(backGetTowerCube()));
    addToMap("backGetSecondCube", MirroredTrajectory(backGetSecondCube()));
    addToMap("backAlign", MirroredTrajectory(backAlign(false)));
    addToMap("backLongAlign", MirroredTrajectory(backAlign(true)));
    addToMap("backSetup", MirroredTrajectory(backSetup()));

    addToMap("pSkillsIntake", MirroredTrajectory(pSkillsIntake()));
    addToMap("pSkillsSetup", MirroredTrajectory(pSkillsSetup()));
    addToMap("pSkillsSecondIntake", MirroredTrajectory(pSkillsSecondIntake()));
    addToMap("longWallBump", MirroredTrajectory(longWallBump()));
    addToMap("alignWithFirstTower", MirroredTrajectory(alignWithFirstTower()));
    addToMap("intakeFirstTower", MirroredTrajectory(intakeFirstTower()));
    addToMap("shortTower", MirroredTrajectory(shortTower(false, false)));
    addToMap("midTower", MirroredTrajectory(shortTower(false, true)));
    addToMap("longTower", MirroredTrajectory(shortTower(true, false)));
    addToMap("backCurve", MirroredTrajectory(backCurve()));

    addToMap("frontIntake", MirroredTrajectory(getFrontIntake()));
    addToMap("frontSetup", MirroredTrajectory(getFrontSetup()));

    addToMap("alignInThirdTower", MirroredTrajectory(alignInThirdTower()));
    addToMap("placeInThirdTower", MirroredTrajectory(placeInThirdTower()));

    addToMap("stackPullBack", MirroredTrajectory(stackPullBack(false)));
    addToMap("skillsStackPullBack", MirroredTrajectory(stackPullBack(true)));

    complete_ = true;
    printf("end\n");
  }
}
