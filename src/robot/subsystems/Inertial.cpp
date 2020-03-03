#include "Inertial.hpp"
#include "../../lib/meecan_lib.hpp"
#include "../Constants.hpp"

namespace subsystems {
  Inertial::InertialManager Inertial::instance;

  Inertial::Inertial() {
    sensor = new pros::Imu(constants::RobotConstants::IMU_PORT);
    sensor->reset();
    offset = 0;
  }
  bool Inertial::ready() {
    return !sensor->is_calibrating();
  }
  void Inertial::resetRotation() {
    offset = sensor->get_rotation();
  }
  units::Angle Inertial::getRotation() {
    double degrees = sensor->get_rotation() - offset;
    return -degrees * units::degree;
  }
  void Inertial::registerEnabledLoops(loops::Looper* enabledLooper) {

  }
  void Inertial::stop() {

  }
}
