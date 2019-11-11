//
// Created by alexweiss on 8/14/19.
//

#ifndef INC_7405M_CODE_SRC_ROBOT_AUTO_ACTIONS_LIFTPOSITION_HPP_
#define INC_7405M_CODE_SRC_ROBOT_AUTO_ACTIONS_LIFTPOSITION_HPP_

#include "Action.hpp"
#include "../../../lib/meecan_lib.hpp"
#include "../../subsystems/Lift.hpp"

namespace auton {
  namespace actions {
    class LiftPosition : public Action {
     private:
      units::QTime start_time_;
      double goal_;

     public:
      LiftPosition(double goal);
      bool isFinished();
      void start();
      void update();
      void done();
    };
  }
}

#endif //INC_7405M_CODE_SRC_ROBOT_AUTO_ACTIONS_LIFTPOSITION_HPP_
