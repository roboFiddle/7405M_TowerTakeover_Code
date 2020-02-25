#ifndef INC_7405M_CODE_SRC_ROBOT_SUBSYSTEMS_VISION_HPP_
#define INC_7405M_CODE_SRC_ROBOT_SUBSYSTEMS_VISION_HPP_

#include "Subsystem.hpp"
#include "main.h"
#include <vector>

namespace subsystems {
  class VisionPoint {
  public:
    int x, y;
    VisionPoint(int a, int b) : x(a), y(b) {}
  };

  class VisionSystem : Subsystem {
   public:
    enum class Camera {SIDE, FORWARD};
    enum class Position {LEFT, CENTER, RIGHT};
    enum class CubeColor {GREEN, ORANGE, PURPLE};
   private:
    pros::Vision* side;
    pros::Vision* forward;
    pros::ADIAnalogIn* intakeSensor;
   public:
    VisionSystem();
    VisionPoint findExtreme(Camera cam, Position where, CubeColor what);
    int findXExtreme(Camera cam, Position where, CubeColor what);
    bool cubeInIntake();
    void registerEnabledLoops(loops::Looper* enabledLooper);
    void stop();
    struct VisionManager : util::Singleton<VisionSystem, VisionManager> {};
    static VisionManager instance;
  };
}

#endif //INC_7405M_CODE_SRC_ROBOT_SUBSYSTEMS_ODOMETRY_HPP_
