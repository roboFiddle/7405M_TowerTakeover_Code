//
// Created by alexweiss on 8/14/19.
//

#ifndef INC_7405M_CODE_SRC_ROBOT_PATHS_DRIVEMOTIONPLANNER_HPP_
#define INC_7405M_CODE_SRC_ROBOT_PATHS_DRIVEMOTIONPLANNER_HPP_

#include "../../lib/meecan_lib.hpp"
#include <vector>
#include <string>

namespace path_planning {
  enum FollowerType {
    FEEDFORWARD_ONLY,
    PURE_PURSUIT,
    PID,
    NONLINEAR_FEEDBACK
  };

  class Output {
   protected:
    Output updatePID(physics::DifferentialDrive::DriveDynamics dynamics, geometry::Pose2d current_state);
    Output updatePurePursuit(physics::DifferentialDrive::DriveDynamics dynamics, geometry::Pose2d current_state);
    Output updateNonlinearFeedback(physics::DifferentialDrive::DriveDynamics dynamics, geometry::Pose2d current_state);
    Output update(units::QTime timestamp, geometry::Pose2d current_state);
   public:
    units::QAngularSpeed left_velocity, right_velocity;
    units::QAngularAcceleration left_accel, right_accel;
    units::Number left_feedforward_voltage, right_feedforward_voltage;

    Output(units::QAngularSpeed left_velocity, units::QAngularSpeed  right_velocity,
           units::QAngularAcceleration left_accel, units::QAngularAcceleration right_accel,
           units::Number left_feedforward_voltage, units::Number right_feedforward_voltage);

    void flip();
    bool isDone();
    geometry::Pose2d error();
    trajectory::TimedState<geometry::Pose2dWithCurvature> setpoint();
  };

  class DriveMotionPlanner {
   private:
    FollowerType mFollowerType_ = FollowerType::NONLINEAR_FEEDBACK;
    physics::DifferentialDrive model_;
    units::QTime dt_;

   public:
    void setFollowerType(FollowerType type);
    DriveMotionPlanner();
    void setTrajectory(trajectory::TrajectoryIterator<trajectory::TimedState<geometry::Pose2dWithCurvature>> trajectory);
    void reset();
    trajectory::Trajectory<trajectory::TimedState<geometry::Pose2dWithCurvature>> generateTrajectory(
        bool reversed,
        std::vector<geometry::Pose2d> waypoints,
        std::vector<trajectory::TimingConstraint<geometry::Pose2dWithCurvature>*> constraints,
        units::QSpeed max_vel,  // inches/s
        units::QAcceleration max_accel,  // inches/s^2
        units::Number max_voltage);

    std::string toCSV();






  };
}

#endif //INC_7405M_CODE_SRC_ROBOT_PATHS_DRIVEMOTIONPLANNER_HPP_
