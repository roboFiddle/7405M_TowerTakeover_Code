//
// Created by alexweiss on 8/15/19.
//

#ifndef INC_7405M_CODE_SRC_ROBOT_ROBOTSTATE_HPP_
#define INC_7405M_CODE_SRC_ROBOT_ROBOTSTATE_HPP_

#include "../lib/meecan_lib.hpp"

namespace meecan {
  class RobotState {
   public:
    geometry::Pose2d getRobotPose();

    struct RobotStateManager : util::Singleton<RobotState, RobotStateManager> {};
    static RobotStateManager instance;
  };
}

#endif //INC_7405M_CODE_SRC_ROBOT_ROBOTSTATE_HPP_
