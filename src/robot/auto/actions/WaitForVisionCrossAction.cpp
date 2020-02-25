 #include "WaitForVisionCrossAction.hpp"

namespace auton {
  namespace actions {
    WaitForVisionCrossAction::WaitForVisionCrossAction(subsystems::VisionSystem::Camera cam, subsystems::VisionSystem::Position pos, subsystems::VisionSystem::CubeColor color, int thresh, Direction dir) {
      camera = cam;
      position = pos;
      cube = color;
      threshold = thresh;
      direction = dir;
      crossed = false;
      activated = false;
    }
    bool WaitForVisionCrossAction::isFinished() {
      return crossed;
    }
    void WaitForVisionCrossAction::start() {

    }
    void WaitForVisionCrossAction::update() {
      if(!activated) {
        if(subsystems::VisionSystem::instance->findXExtreme(camera, position, cube) != -500) {
          activated = true;
        }
        return;
      }
      int cube_pos = subsystems::VisionSystem::instance->findXExtreme(camera, position, cube);
      if(direction == Direction::FROM_BELOW) {
        crossed = cube_pos > threshold;
      }
      else if(direction == Direction::FROM_ABOVE) {
        crossed = cube_pos < threshold;
      }
    }
    void WaitForVisionCrossAction::done() {

    }
  }
}
