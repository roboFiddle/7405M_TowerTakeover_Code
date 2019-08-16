//
// Created by alexweiss on 8/15/19.
//

#include "../Constants.hpp"
#include "Odometry.hpp"
#include <stdio.h>

namespace subsystems {
  Odometry::OdometryManager Odometry::instance;
  Odometry::Odometry() {
    left = new pros::ADIEncoder(1, 2, true);
    right = new pros::ADIEncoder(3, 4, true);
    back = new pros::ADIEncoder(5, 6);
  }
  void Odometry::updatePosition() {
    units::QLength leftTravel = (left->get_value()) * (M_PI / 180.0) * constants::RobotConstants::kDeadwheelRadius;
    units::QLength rightTravel = (right->get_value()) * (M_PI / 180.0) * constants::RobotConstants::kDeadwheelRadius;
    units::QLength backTravel = (back->get_value()) * (M_PI / 180.0) * constants::RobotConstants::kDeadwheelRadius;

    left->reset();
    right->reset();
    back->reset();

    units::Number dTheta = (rightTravel - leftTravel) / (constants::RobotConstants::kDeadwheelBaseWidth);
    units::QLength dX =  (0.5 * (leftTravel + rightTravel));
    units::QLength dY = backTravel - constants::RobotConstants::kDeadwheelTurnRadius *  dTheta;

    geometry::Twist2d delta(dX, dY, dTheta * units::radian);
    geometry::Pose2d change = geometry::Pose2d::exp(delta);

    printf("%f %f %f\n", change.translation().x(), change.translation().y(), change.rotation().getDegrees());

    currentPosition = currentPosition.transformBy(change);
  }
  geometry::Pose2d Odometry::getPosition() {
    return currentPosition;
  }
  void Odometry::registerEnabledLoops(loops::Looper* enabledLooper) {
    loops::Loop* thisLoop = new loops::Loop();
    thisLoop->onStart = []() {};
    thisLoop->onLoop = [] () { Odometry::instance->updatePosition(); };
    thisLoop->onDisable = []() {};
    enabledLooper->add(std::shared_ptr<loops::Loop>(thisLoop));
  }
  void Odometry::stop() {

  }
}
