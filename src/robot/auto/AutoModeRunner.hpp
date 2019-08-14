//
// Created by alexweiss on 8/14/19.
//

#ifndef INC_7405M_CODE_SRC_ROBOT_AUTO_AUTOMODERUNNER_HPP_
#define INC_7405M_CODE_SRC_ROBOT_AUTO_AUTOMODERUNNER_HPP_

#include "modes/AutoModeBase.hpp"
#include "../loops/Looper.hpp"
#include "../loops/Loop.hpp"
#include <memory>

namespace auton {
  class AutoModeRunner {
   private:
    std::shared_ptr<AutoModeBase> mode;
    loops::Looper* loop;
   public:
    void setAutoMode(std::shared_ptr<AutoModeBase> new_auto_mode);
    void start();
    void stop();
    AutoModeBase* getAutoMode();

  };
}

#endif //INC_7405M_CODE_SRC_ROBOT_AUTO_AUTOMODERUNNER_HPP_
