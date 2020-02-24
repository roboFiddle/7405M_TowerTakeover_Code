
#ifndef INC_7405M_CODE_SRC_ROBOT_AUTO_ACTIONS_WAITVISIONACTION_HPP_
#define INC_7405M_CODE_SRC_ROBOT_AUTO_ACTIONS_WAITVISIONACTION_HPP_

#include "Action.hpp"
#include "../../subsystems/VisionSystem.hpp"

namespace auton {
  namespace actions {
    class WaitForVisionCrossAction : public Action {
     public:
      enum class Direction {FROM_BELOW, FROM_ABOVE};
     private:
      subsystems::VisionSystem::Camera camera;
      subsystems::VisionSystem::Position position;
      subsystems::VisionSystem::CubeColor cube;
      int threshold;
      Direction direction;
      int activated;
      bool crossed;
     public:
      WaitForVisionCrossAction(subsystems::VisionSystem::Camera cam, subsystems::VisionSystem::Position pos, subsystems::VisionSystem::CubeColor color, int thresh, Direction dir);
      bool isFinished();
      void start();
      void update();
      void done();
    };
  }
}
#endif //INC_7405M_CODE_SRC_ROBOT_AUTO_ACTIONS_WAITACTION_HPP_
