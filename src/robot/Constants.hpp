//
// Created by alexweiss on 8/9/19.
//

#ifndef INC_7405M_CODE_SRC_ROBOT_CONSTANTS_HPP_
#define INC_7405M_CODE_SRC_ROBOT_CONSTANTS_HPP_

#include "../lib/utility/Units.hpp"

namespace constants {
  class RobotConstants {
   public:
    static constexpr units::QLength kDriveWheelRadius = 2*units::inch;
    static constexpr units::QMass kRobotMass = 4*units::kg;
    static constexpr units::QMoment kRobotMoment = 6*units::kgm2;
    static constexpr units::QAngularDrag kRobotAngularDrag = 0.0;
    static constexpr units::QLength kDriveWheelTrackWidth = 14.875 * units::inch;
    static constexpr units::Number kTrackScrubFactor = 1.0;

    static constexpr units::QLength kDeadwheelRadius = 1.375*units::inch;
    static constexpr units::QLength kDeadwheelBaseWidth = 8.5*units::inch;
    static constexpr units::QLength kDeadwheelTurnRadius = 6.75*units::inch;

    static constexpr units::QAngularSpeed kDriveSpeedPerVolt = 1.09; // RADIANS / SECOND / VOTL
    static constexpr units::QTorque kDriveTorquePerVolt = 0.56; // Nm / VOLT
    static constexpr units::Number kDriveFrictionVoltage = 1.0; // voltage to overcome friction (V)

  };
}

#endif //INC_7405M_CODE_SRC_ROBOT_CONSTANTS_HPP_
