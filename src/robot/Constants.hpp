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
    static constexpr units::QLength kDeadwheelBaseWidth = 9*units::inch;

    static constexpr units::QAngularSpeed kDriveSpeedPerVolt = 1.09; // RADIANS / SECOND / VOTL
    static constexpr units::QTorque kDriveTorquePerVolt = 0.56; // Nm / VOLT
    static constexpr units::Number kDriveFrictionVoltage = 1.0; // voltage to overcome friction (V)

    static constexpr double turnKP = 125;
    static constexpr double turnKI = 0;
    static constexpr double turnKD = 1;

    static constexpr int motor_drive_frontleft = 4;
    static constexpr int motor_drive_backleft = 3;
    static constexpr int motor_drive_frontright = 6;
    static constexpr int motor_drive_backright = 5;
    static constexpr int motor_intake_left = 1;
    static constexpr int motor_intake_right = 16;
    static constexpr int motor_tray = 8;
    static constexpr int motor_lift = 14;

    static constexpr double MAX_TRAY_RPM = 65;
    static constexpr double MAX_LIFT_RPM = 150;

    static constexpr double LIFT_STAGE[2] = {-10, -900};
    static constexpr double TRAY_LIFT[3] = {0.0, 575, 800};
    static constexpr double LIFT_PRESETS[3] = {-900, -1100, -1700};
    static constexpr double trayErrorBeforeLiftStart = 25;

  };
}

#endif //INC_7405M_CODE_SRC_ROBOT_CONSTANTS_HPP_
