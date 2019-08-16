//
// Created by alexweiss on 8/14/19.
//

#ifndef INC_7405M_CODE_SRC_ROBOT_SUBSYSTEMS_DRIVE_HPP_
#define INC_7405M_CODE_SRC_ROBOT_SUBSYSTEMS_DRIVE_HPP_

#include "../paths/PathFollower.hpp"
#include "Subsystem.hpp"
#include "../../lib/meecan_lib.hpp"
#include "main.h"

namespace subsystems {
  enum DriveControlState {
    OPEN_LOOP, // open loop voltage control
    PATH_FOLLOWING, // velocity PID control
  };

  class Drive : Subsystem {
   private:
    pros::Motor* frontLeft;
    pros::Motor* frontRight;
    pros::Motor* backLeft;
    pros::Motor* backRight;
    DriveControlState currentState;
    path_planning::PathFollower* currentFollower;
    units::QTime startTime;
    bool forceStopTrajectory_;
    double left_demand, right_demand;
    double left_accel, right_accel;
    double left_feed_forward, right_feed_forward;

   public:
    Drive();
    void stop() ;
    void zeroSensors();
    void setOpenLoop(util::DriveSignal voltage);
    void setVelocity(util::DriveSignal velocity, util::DriveSignal feedforward);
    void registerEnabledLoops(loops::Looper* enabledLooper);
    void setTrajectory(trajectory::TrajectoryIterator<trajectory::TimedState<geometry::Pose2dWithCurvature>> trajectory);
    void updatePathFollower();
    void updateOutputs();
    void setBrakeMode(bool set);
    DriveControlState getState();
    bool isDoneWithTrajectory();
    void overrideTrajectory();


    struct DriveManager : util::Singleton<Drive, DriveManager> {};
    static DriveManager instance;
  };
}

#endif //INC_7405M_CODE_SRC_ROBOT_SUBSYSTEMS_DRIVE_HPP_
