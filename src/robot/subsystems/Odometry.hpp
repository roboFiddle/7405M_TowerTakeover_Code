//
// Created by alexweiss on 8/15/19.
//

#ifndef INC_7405M_CODE_SRC_ROBOT_SUBSYSTEMS_ODOMETRY_HPP_
#define INC_7405M_CODE_SRC_ROBOT_SUBSYSTEMS_ODOMETRY_HPP_

#include "Subsystem.hpp"
#include "main.h"

namespace subsystems {
  class Odometry : Subsystem {
   private:
    pros::ADIEncoder* left;
    pros::ADIEncoder* right;
    pros::ADIEncoder* back;
    geometry::Pose2d currentPosition;
   public:
    Odometry();
    void updatePosition();
    geometry::Pose2d getPosition();
    void registerEnabledLoops(loops::Looper* enabledLooper);
    void stop();

    struct OdometryManager : util::Singleton<Odometry, OdometryManager> {};
    static OdometryManager instance;
  };
}

#endif //INC_7405M_CODE_SRC_ROBOT_SUBSYSTEMS_ODOMETRY_HPP_
