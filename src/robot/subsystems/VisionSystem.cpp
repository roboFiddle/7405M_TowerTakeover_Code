#include "VisionSystem.hpp"
#include "../Constants.hpp"

#include <stdio.h>
#include <cmath>

namespace subsystems {
  VisionSystem::VisionManager VisionSystem::instance;

  VisionSystem::VisionSystem() {
    side = new pros::Vision(constants::RobotConstants::SIDE_VISION_PORT, pros::E_VISION_ZERO_CENTER);
    forward = new pros::Vision(constants::RobotConstants::FORWARD_VISION_PORT, pros::E_VISION_ZERO_CENTER);
    intakeSensor = new pros::ADIAnalogIn(5);

    pros::vision_signature_s_t GREEN_SIG = pros::Vision::signature_from_utility(0, 8973, 11143, 10058, -2119, -1053, -1586, 5.4, 0);
    pros::vision_signature_s_t ORANGE_SIG = pros::Vision::signature_from_utility(constants::RobotConstants::ORANGE_SIG, 8973, 11143, 10058, -2119, -1053, -1586, 5.4, 0);
    pros::vision_signature_s_t PURPLE_SIG = pros::Vision::signature_from_utility(constants::RobotConstants::PURPLE_SIG, 8973, 11143, 10058, -2119, -1053, -1586, 5.4, 0);

    side->set_signature(1, &GREEN_SIG);
    side->set_signature(2, &ORANGE_SIG);
    side->set_signature(3, &PURPLE_SIG);
    forward->set_signature(1, &GREEN_SIG);
    forward->set_signature(2, &ORANGE_SIG);
    forward->set_signature(3, &PURPLE_SIG);
  }
  VisionPoint VisionSystem::findExtreme(Camera cam, Position where, CubeColor what) {
    pros::vision_object_s_t object_arr[8];
    int sig;
    switch(what) {
      case CubeColor::GREEN:
        sig = 1;
        break;
      case CubeColor::ORANGE:
        sig = 2;
        break;
      case CubeColor::PURPLE:
        sig = 3;
        break;
      default:
        sig = 1;
        break;
    }
    int actual_count;
    if(cam == Camera::SIDE)
      actual_count = side->read_by_code(0, sig, 8, object_arr);
    else
      actual_count = forward->read_by_code(0, sig, 8, object_arr);

    if(actual_count == 0)
      return VisionPoint(-500, 0);

    int extremeValue = (where == Position::RIGHT ? -500 : 500);
    int extremeIndex = -1;
    for(int i = 0; i < actual_count; i++) {
      int x_position = object_arr[i].x_middle_coord;
      switch(where) {
        case Position::LEFT:
          if(x_position < extremeValue) {
            extremeValue = x_position;
            extremeIndex = i;
          }
          break;
        case Position::CENTER:
          if(std::abs(x_position) < std::abs(extremeValue)) {
            extremeValue = x_position;
            extremeIndex = i;
          }
          break;
        case Position::RIGHT:
          if(x_position > extremeValue) {
            extremeValue = x_position;
            extremeIndex = i;
          }
          break;
      }
    }
    return VisionPoint(object_arr[extremeIndex].x_middle_coord, object_arr[extremeIndex].y_middle_coord);
  }
  int VisionSystem::findXExtreme(Camera cam, Position where, CubeColor what) {
    return findExtreme(cam, where, what).x;
  }
  bool VisionSystem::cubeInIntake() {
    return intakeSensor->get_value() > 2000;
  }
  void VisionSystem::registerEnabledLoops(loops::Looper* enabledLooper) {

  }
  void VisionSystem::stop() {

  }

}
