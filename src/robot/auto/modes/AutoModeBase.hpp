//
// Created by alexweiss on 8/14/19.
//

#ifndef INC_7405M_CODE_SRC_ROBOT_AUTO_AUTOMODEBASE_HPP_
#define INC_7405M_CODE_SRC_ROBOT_AUTO_AUTOMODEBASE_HPP_

#include "../actions/Action.hpp"

namespace auton {
  class AutoModeBase {
   protected:
    virtual void routine() = 0;
   public:
    virtual void run() = 0;
    virtual void done() = 0;
    void stop();
    bool isActive();
    void runAction(actions::Action* action);
  };
}

#endif //INC_7405M_CODE_SRC_ROBOT_AUTO_AUTOMODEBASE_HPP_
