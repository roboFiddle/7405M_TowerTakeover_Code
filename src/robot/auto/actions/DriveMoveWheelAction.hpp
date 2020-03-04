//
// Created by alexweiss on 11/10/19.
//

#ifndef INC_7405M_CODE_SRC_ROBOT_AUTO_ACTIONS_DRIVETURNMOVELACTION_HPP_
#define INC_7405M_CODE_SRC_ROBOT_AUTO_ACTIONS_DRIVETURNMOVELACTION_HPP_

#include "Action.hpp"
#include "../../../lib/meecan_lib.hpp"
#include "../../subsystems/Tray.hpp"

namespace auton {
  namespace actions {
    class DriveMoveWheelAction : public Action {
      private:
        units::QLength dist_;
        bool s_;

      public:
        DriveMoveWheelAction(units::QLength distance, bool s = false);
        bool isFinished();
        void start();
        void update();
        void done();
    };
  }
}

#endif //INC_7405M_CODE_SRC_ROBOT_AUTO_ACTIONS_OPENLOOPLIFTEACTION_HPP_
