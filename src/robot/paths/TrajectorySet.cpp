//
// Created by alexweiss on 8/14/19.
//

#include "TrajectorySet.hpp"
#include "DriveMotionPlanner.hpp"
#include "PathConstants.hpp"

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
  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::getBackForward() {
    std::vector<geometry::Pose2d> waypoints;
    waypoints.push_back(geometry::Pose2d(0, 0, geometry::Rotation2d::fromDegrees(0)));
    waypoints.push_back(geometry::Pose2d(30 * units::inch, 0, geometry::Rotation2d::fromDegrees(0)));

    std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> noConstraints;

    return DriveMotionPlanner::generateTrajectory(false, waypoints, noConstraints,
        constants::PathConstants::kMaxVelocity * 0.5, constants::PathConstants::kMaxAccel, 8.0);
  }
  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::getBackFastForward() {
    std::vector<geometry::Pose2d> waypoints;
    waypoints.push_back(geometry::Pose2d(0, 0, geometry::Rotation2d::fromDegrees(0)));
    waypoints.push_back(geometry::Pose2d(33 * units::inch, 0, geometry::Rotation2d::fromDegrees(0)));

    std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> noConstraints;

    return DriveMotionPlanner::generateTrajectory(false, waypoints, noConstraints,
        constants::PathConstants::kMaxVelocity * 1, constants::PathConstants::kMaxAccel, 8.0);
  }

  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::getBackSCurve() {
    std::vector<geometry::Pose2d> waypoints;
    waypoints.push_back(geometry::Pose2d(0, 0, geometry::Rotation2d::fromDegrees(0)));
    waypoints.push_back(geometry::Pose2d(18 * units::inch, 0 * units::inch, geometry::Rotation2d::fromDegrees(0)));

    std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> noConstraints;

    return DriveMotionPlanner::generateTrajectory(false, waypoints, noConstraints,
        constants::PathConstants::kMaxVelocity * 1, constants::PathConstants::kMaxAccel, 8.0);
  }

  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::getFrontForward() {
    std::vector<geometry::Pose2d> waypoints;
    waypoints.push_back(geometry::Pose2d(0, 0, geometry::Rotation2d::fromDegrees(0)));
    waypoints.push_back(geometry::Pose2d(30 * units::inch, 0, geometry::Rotation2d::fromDegrees(0)));

    std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> noConstraints;

    return DriveMotionPlanner::generateTrajectory(false, waypoints, noConstraints,
                                                  constants::PathConstants::kMaxVelocity * 0.5, constants::PathConstants::kMaxAccel, 8.0);
  }

  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::getPSkillsForward() {
    std::vector<geometry::Pose2d> waypoints;
    waypoints.push_back(geometry::Pose2d(0, 0, geometry::Rotation2d::fromDegrees(0)));
    waypoints.push_back(geometry::Pose2d(94 * units::inch, 0 * units::inch, geometry::Rotation2d::fromDegrees(0)));

    std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> noConstraints;

    return DriveMotionPlanner::generateTrajectory(false, waypoints, noConstraints,
        constants::PathConstants::kMaxVelocity * 0.5, constants::PathConstants::kMaxAccel, 8.0);
  }
  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::getBackS() {
    std::vector<geometry::Pose2d> waypoints;

    waypoints.push_back(geometry::Pose2d(0 * units::inch, 0 * units::inch, geometry::Rotation2d::fromDegrees(0)));
    waypoints.push_back(geometry::Pose2d(26 * units::inch, 0 * units::inch, geometry::Rotation2d::fromDegrees(0)));

    std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> noConstraints;

    return DriveMotionPlanner::generateTrajectory(false, waypoints, noConstraints,
        constants::PathConstants::kMaxVelocity, constants::PathConstants::kMaxAccel, 8.0);
  }
  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::getFrontS() {
    std::vector<geometry::Pose2d> waypoints;

    waypoints.push_back(geometry::Pose2d(0 * units::inch, 0 * units::inch, geometry::Rotation2d::fromDegrees(0)));
    waypoints.push_back(geometry::Pose2d(18 * units::inch, 0 * units::inch, geometry::Rotation2d::fromDegrees(0)));

    std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> noConstraints;

    return DriveMotionPlanner::generateTrajectory(false, waypoints, noConstraints,
                                                  constants::PathConstants::kMaxVelocity, constants::PathConstants::kMaxAccel, 8.0);
  }
  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::getFrontSetup() {
    std::vector<geometry::Pose2d> waypoints;

    waypoints.push_back(geometry::Pose2d(0 * units::inch, 0 * units::inch, geometry::Rotation2d::fromDegrees(0)));
    waypoints.push_back(geometry::Pose2d(43 * units::inch, 0 * units::inch, geometry::Rotation2d::fromDegrees(0)));

    std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> noConstraints;

    return DriveMotionPlanner::generateTrajectory(false, waypoints, noConstraints,
        constants::PathConstants::kMaxVelocity * 0.65, constants::PathConstants::kMaxAccel, 8.0);
  }
  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::getBackSetup() {
    std::vector<geometry::Pose2d> waypoints;

    waypoints.push_back(geometry::Pose2d(0 * units::inch, 0 * units::inch, geometry::Rotation2d::fromDegrees(0)));
    waypoints.push_back(geometry::Pose2d(10.5 * units::inch, 0 * units::inch, geometry::Rotation2d::fromDegrees(0)));

    std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> noConstraints;

    return DriveMotionPlanner::generateTrajectory(false, waypoints, noConstraints,
        constants::PathConstants::kMaxVelocity, constants::PathConstants::kMaxAccel, 8.0);
  }
  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::getPSetup() {
    std::vector<geometry::Pose2d> waypoints;

    waypoints.push_back(geometry::Pose2d(0 * units::inch, 0 * units::inch, geometry::Rotation2d::fromDegrees(0)));
    waypoints.push_back(geometry::Pose2d(17 * units::inch, 0 * units::inch, geometry::Rotation2d::fromDegrees(0)));

    std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> noConstraints;

    return DriveMotionPlanner::generateTrajectory(false, waypoints, noConstraints,
        constants::PathConstants::kMaxVelocity, constants::PathConstants::kMaxAccel, 8.0);
  }
  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::getPFirstTower() {
    std::vector<geometry::Pose2d> waypoints;

    waypoints.push_back(geometry::Pose2d(0 * units::inch, 0 * units::inch, geometry::Rotation2d::fromDegrees(0)));
    waypoints.push_back(geometry::Pose2d(32 * units::inch, 0 * units::inch, geometry::Rotation2d::fromDegrees(0)));

    std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> noConstraints;

    return DriveMotionPlanner::generateTrajectory(false, waypoints, noConstraints,
        constants::PathConstants::kMaxVelocity * 0.8, constants::PathConstants::kMaxAccel, 8.0);
  }
  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::getPBackFirstTower() {
    std::vector<geometry::Pose2d> waypoints;

    waypoints.push_back(geometry::Pose2d(0 * units::inch, 0 * units::inch, geometry::Rotation2d::fromDegrees(0)));
    waypoints.push_back(geometry::Pose2d(5 * units::inch, 0 * units::inch, geometry::Rotation2d::fromDegrees(0)));

    std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> noConstraints;

    return DriveMotionPlanner::generateTrajectory(false, waypoints, noConstraints,
        constants::PathConstants::kMaxVelocity, constants::PathConstants::kMaxAccel, 8.0);
  }
  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::getPIntakeSlowFirstTower() {
    std::vector<geometry::Pose2d> waypoints;

    waypoints.push_back(geometry::Pose2d(0 * units::inch, 0 * units::inch, geometry::Rotation2d::fromDegrees(0)));
    waypoints.push_back(geometry::Pose2d(5 * units::inch, 0 * units::inch, geometry::Rotation2d::fromDegrees(0)));

    std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> noConstraints;

    return DriveMotionPlanner::generateTrajectory(false, waypoints, noConstraints,
        constants::PathConstants::kMaxVelocity * 0.5, constants::PathConstants::kMaxAccel, 8.0);
  }
  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::getPurePursuitTest() {
    std::vector<geometry::Pose2d> waypoints;

    waypoints.push_back(geometry::Pose2d(0 * units::inch, 0 * units::inch, geometry::Rotation2d::fromDegrees(0)));
    waypoints.push_back(geometry::Pose2d(36 * units::inch, 36 * units::inch, geometry::Rotation2d::fromDegrees(0)));

    std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> noConstraints;

    return DriveMotionPlanner::generateTrajectory(false, waypoints, noConstraints,
        constants::PathConstants::kMaxVelocity, constants::PathConstants::kMaxAccel, 8.0);
  }
  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::getStackPullBack() {
    std::vector<geometry::Pose2d> waypoints;
    waypoints.push_back(geometry::Pose2d(0, 0, geometry::Rotation2d::fromDegrees(0)));
    waypoints.push_back(geometry::Pose2d(15 * units::inch, 0, geometry::Rotation2d::fromDegrees(0)));

    std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> noConstraints;

    return trajectory::TimingUtil::reverseTimed(DriveMotionPlanner::generateTrajectory(false, waypoints, noConstraints,
        constants::PathConstants::kMaxVelocity * 1.5, constants::PathConstants::kMaxAccel * 2, 8.0));
  }

  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::getPSkillsLongBackSecondTower() {
    std::vector<geometry::Pose2d> waypoints;
    waypoints.push_back(geometry::Pose2d(0, 0, geometry::Rotation2d::fromDegrees(0)));
    waypoints.push_back(geometry::Pose2d(16 * units::inch, 0, geometry::Rotation2d::fromDegrees(0)));

    std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> noConstraints;

    return DriveMotionPlanner::generateTrajectory(false, waypoints, noConstraints,
                                                                                       constants::PathConstants::kMaxVelocity, constants::PathConstants::kMaxAccel, 8.0);
  }

  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::getPSkillsIntakeSecondTower() {
    std::vector<geometry::Pose2d> waypoints;
    waypoints.push_back(geometry::Pose2d(0, 0, geometry::Rotation2d::fromDegrees(0)));
    waypoints.push_back(geometry::Pose2d(12 * units::inch, 0, geometry::Rotation2d::fromDegrees(0)));

    std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> noConstraints;

    return trajectory::TimingUtil::reverseTimed(DriveMotionPlanner::generateTrajectory(false, waypoints, noConstraints,
                                                                                       constants::PathConstants::kMaxVelocity, constants::PathConstants::kMaxAccel, 8.0));
  }

  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::getPSkillsLineUpBackSecondStack() {
    std::vector<geometry::Pose2d> waypoints;
    waypoints.push_back(geometry::Pose2d(0, 0, geometry::Rotation2d::fromDegrees(0)));
    waypoints.push_back(geometry::Pose2d(15 * units::inch, 0, geometry::Rotation2d::fromDegrees(0)));

    std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> noConstraints;

    return trajectory::TimingUtil::reverseTimed(DriveMotionPlanner::generateTrajectory(false, waypoints, noConstraints,
                                                                                       constants::PathConstants::kMaxVelocity, constants::PathConstants::kMaxAccel, 8.0));
  }

  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::getPSkillsLineUpForwardSecondStack() {
    std::vector<geometry::Pose2d> waypoints;
    waypoints.push_back(geometry::Pose2d(0, 0, geometry::Rotation2d::fromDegrees(0)));
    waypoints.push_back(geometry::Pose2d(20 * units::inch, 0, geometry::Rotation2d::fromDegrees(0)));

    std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> noConstraints;

    return trajectory::TimingUtil::reverseTimed(DriveMotionPlanner::generateTrajectory(false, waypoints, noConstraints,
                                                                                       constants::PathConstants::kMaxVelocity, constants::PathConstants::kMaxAccel, 8.0));
  }

  trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> TrajectorySet::getPSkillsIntakeLastTower() {
    std::vector<geometry::Pose2d> waypoints;
    waypoints.push_back(geometry::Pose2d(0, 0, geometry::Rotation2d::fromDegrees(0)));
    waypoints.push_back(geometry::Pose2d(40 * units::inch, 0, geometry::Rotation2d::fromDegrees(0)));

    std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> noConstraints;

    return trajectory::TimingUtil::reverseTimed(DriveMotionPlanner::generateTrajectory(false, waypoints, noConstraints,
                                                                                       constants::PathConstants::kMaxVelocity, constants::PathConstants::kMaxAccel, 8.0));
  }

  void TrajectorySet::generatorCalls() {
    printf("start\n");
    addToMap("backForward", MirroredTrajectory(getBackForward()));
    addToMap("backFastForward", MirroredTrajectory(getBackFastForward()));
    addToMap("frontForward", MirroredTrajectory(getBackForward()));
    addToMap("backS", MirroredTrajectory(getBackS()));
    addToMap("backSCurve", MirroredTrajectory(getBackSCurve()));
    addToMap("frontSetup", MirroredTrajectory(getFrontSetup()));
    addToMap("backSetup", MirroredTrajectory(getBackSetup()));
    addToMap("stackPullBack", MirroredTrajectory(getStackPullBack()));
    addToMap("pSkillsIntakeLastTower", MirroredTrajectory(getPIntakeSlowFirstTower()));
    addToMap("pptest", MirroredTrajectory(getPurePursuitTest()));

    /* addToMap("programmingSkillsForward", MirroredTrajectory(getPSkillsForward()));
    addToMap("programmingSkillsSetup", MirroredTrajectory(getPSetup()));
    addToMap("pSkillsIntakeFirstTower", MirroredTrajectory(getPFirstTower()));
    addToMap("pSkillsBackTower", MirroredTrajectory(getPBackFirstTower()));
    addToMap("pSkillsLongBackSecondTower", MirroredTrajectory(getPSkillsLongBackSecondTower()));
    addToMap("pSkillsIntakeSecondTower", MirroredTrajectory(getPSkillsIntakeSecondTower()));
    addToMap("pSkillsLineUpBackSecondStack", MirroredTrajectory(getPSkillsLineUpBackSecondStack()));
    addToMap("pSkillsLineUpForwardSecondStack", MirroredTrajectory(getPSkillsLineUpForwardSecondStack()));

    addToMap("pSkillsIntakeCloseTower", MirroredTrajectory(getPIntakeSlowFirstTower())); */
    complete_ = true;
    printf("end\n");
  }
}
