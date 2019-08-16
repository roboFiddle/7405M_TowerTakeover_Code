//
// Created by alexweiss on 8/14/19.
//

#ifndef INC_7405M_CODE_SRC_ROBOT_PATHS_PATHCONSTANTS_HPP_
#define INC_7405M_CODE_SRC_ROBOT_PATHS_PATHCONSTANTS_HPP_

#include "../../lib/meecan_lib.hpp"

namespace constants {
  class PathConstants {
   public:
    static constexpr units::QLength kMaxDx = 0.1;
    static constexpr units::QLength kMaxDy = 0.0l;
    static constexpr units::Angle kMaxDtheta = 0.05;

    static constexpr units::QSpeed kMaxVelocity = 1.0;
    static constexpr units::QAcceleration kMaxAccel = 2.0;
    static constexpr units::QAngularAcceleration kMaxCentripetalAccel = 0.75;
    static constexpr units::Number kMaxVoltage = 12.0;
  };

}

#endif //INC_7405M_CODE_SRC_ROBOT_PATHS_PATHCONSTANTS_HPP_
