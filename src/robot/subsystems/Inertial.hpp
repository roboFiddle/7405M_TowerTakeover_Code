//
// Created by alexweiss on 8/15/19.
//

#ifndef INC_7405M_CODE_SRC_ROBOT_SUBSYSTEMS_INERTIAL_HPP_
#define INC_7405M_CODE_SRC_ROBOT_SUBSYSTEMS_INERTIAL_HPP_

#include "Subsystem.hpp"
#include "main.h"

namespace subsystems {
  class Inertial : Subsystem {
   private:
    pros::Imu* sensor;
    double offset;
   public:
    Inertial();
    bool ready();
    void resetRotation();
    units::Angle getRotation();
    void registerEnabledLoops(loops::Looper* enabledLooper);
    void stop();

    struct InertialManager : util::Singleton<Inertial, InertialManager> {};
    static InertialManager instance;
  };
}

#endif //INC_7405M_CODE_SRC_ROBOT_SUBSYSTEMS_ODOMETRY_HPP_
