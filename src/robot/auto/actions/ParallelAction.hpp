//
// Created by alexweiss on 8/14/19.
//

#ifndef INC_7405M_CODE_SRC_ROBOT_AUTO_ACTIONS_PARALLELACTION_HPP_
#define INC_7405M_CODE_SRC_ROBOT_AUTO_ACTIONS_PARALLELACTION_HPP_

#include <vector>
#include <memory>
#include "Action.hpp"

namespace auton {
  namespace actions {
    class ParallelAction {
     private:
      std::vector<std::shared_ptr<Action>> actions_;
     public:
      ParallelAction(std::vector<std::shared_ptr<Action>> actions);
      bool isFinished();
      void start();
      void update();
      void done();
    };
  }
}

#endif //INC_7405M_CODE_SRC_ROBOT_AUTO_ACTIONS_PARALLELACTION_HPP_
