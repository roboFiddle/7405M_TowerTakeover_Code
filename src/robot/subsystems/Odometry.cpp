//
// Created by alexweiss on 8/15/19.
//

#include "../Constants.hpp"
#include "Odometry.hpp"
#include <stdio.h>

namespace subsystems {
  Odometry::OdometryManager Odometry::instance;
  Odometry::Odometry() {
    left = new pros::ADIEncoder(3, 4, true);
    right = new pros::ADIEncoder(7, 8, true);
    back = new pros::ADIEncoder(5, 6, true);
  }
  void Odometry::updatePosition() {
    units::QLength leftTravel = -(left->get_value()) * 0.0174533 * constants::RobotConstants::kDeadwheelRadius; // .0174533 = PI/180
    units::QLength rightTravel = -(right->get_value()) * 0.0174533 * constants::RobotConstants::kDeadwheelRadius;
    units::QLength backTravel = (back->get_value()) * 0.0174533 * constants::RobotConstants::kDeadwheelRadius;

    units::Number dTheta = (rightTravel - leftTravel) / (constants::RobotConstants::kDeadwheelBaseWidth);
    units::QLength dY =  (0.5 * (leftTravel + rightTravel));
    units::QLength dX = backTravel + constants::RobotConstants::kDeadwheelBaseWidth *  dTheta;

    geometry::Twist2d delta(dX, dY, dTheta * units::radian);
    geometry::Pose2d change = geometry::Pose2d::exp(delta);

    left->reset();
    right->reset();
    back->reset();
    currentPosition = currentPosition.transformBy(change);


    printf("X %f %f %f \n", leftTravel, rightTravel, backTravel);




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
