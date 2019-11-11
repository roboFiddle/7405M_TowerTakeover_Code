//
// Created by alexweiss on 8/14/19.
//

#ifndef INC_7405M_CODE_SRC_ROBOT_AUTO_ACTIONS_TRAYPOSITION_HPP_
#define INC_7405M_CODE_SRC_ROBOT_AUTO_ACTIONS_TRAYPOSITION_HPP_

#include "Action.hpp"
#include "../../../lib/meecan_lib.hpp"
#include "../../subsystems/Tray.hpp"

namespace auton {
  namespace actions {
    class TrayPosition : public Action {
     private:
      units::QTime start_time_;
      double goal_;

     public:
      TrayPosition(double goal);
      bool isFinished();
      void start();
      void update();
      void done();
    };
  }
}

#endif //INC_7405M_CODE_SRC_ROBOT_AUTO_ACTIONS_LIFTPOSITION_HPP_
