//
// Created by alexweiss on 8/14/19.
//

#ifndef INC_7405M_CODE_SRC_ROBOT_PATHS_PATHCONSTANTS_HPP_
#define INC_7405M_CODE_SRC_ROBOT_PATHS_PATHCONSTANTS_HPP_

#include "../../lib/meecan_lib.hpp"

namespace constants {
  class PathConstants {
    static constexpr units::QLength maxDx = 0.05;
    static constexpr units::QLength maxDy = 0.0l;
    static constexpr units::Angle maxDtheta = 0.05;

    static constexpr units::QSpeed kMaxVelocity = 1.0;
    static constexpr units::QAcceleration kMaxAccel = 2.0;
    static constexpr units::QAngularAcceleration kMaxCentripetalAccel = 0.75;
    static constexpr units::Number kMaxVoltage = 12.0;
  };

  class CriticalPoses {

  };

}

#endif //INC_7405M_CODE_SRC_ROBOT_PATHS_PATHCONSTANTS_HPP_
