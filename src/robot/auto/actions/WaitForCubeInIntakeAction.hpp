
#ifndef INC_7405M_CODE_SRC_ROBOT_AUTO_ACTIONS_WAITVISIONCUBEINTAKEACTION_HPP_
#define INC_7405M_CODE_SRC_ROBOT_AUTO_ACTIONS_WAITVISIONCUBEINTAKEACTION_HPP_

#include "Action.hpp"
#include "../../subsystems/VisionSystem.hpp"

namespace auton {
  namespace actions {
    class WaitForCubeInIntakeAction : public Action {
     private:
      bool activated;
     public:
      WaitForCubeInIntakeAction();
      bool isFinished();
      void start();
      void update();
      void done();
    };
  }
}
#endif //INC_7405M_CODE_SRC_ROBOT_AUTO_ACTIONS_WAITACTION_HPP_
