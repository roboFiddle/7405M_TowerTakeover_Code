#include "Constants.hpp"

namespace constants {
    units::QLength RobotConstants::kDriveWheelRadius = 2*units::inch;
    units::QMass RobotConstants::kRobotMass = 4*units::kg;
    units::QMoment RobotConstants::kRobotMoment = 6*units::kgm2;
    units::QAngularDrag RobotConstants::kRobotAngularDrag = 0.0;
    units::QLength RobotConstants::kDriveWheelTrackWidth = 14.875 * units::inch;
    units::Number RobotConstants::kTrackScrubFactor = 1.0;

    units::QLength RobotConstants::kDeadwheelRadius = 1.375*units::inch;
    units::QLength RobotConstants::kDeadwheelBaseWidth = 9*units::inch;

    units::QAngularSpeed RobotConstants::kDriveSpeedPerVolt = 1.09; // RADIANS / SECOND / VOTL
    units::QTorque RobotConstants::kDriveTorquePerVolt = 0.56; // Nm / VOLT
    units::Number RobotConstants::kDriveFrictionVoltage = 1.0; // voltage to overcome friction (V)

    double RobotConstants::turnKP = 125;
    double RobotConstants::turnKI = 0;
    double RobotConstants::turnKD = 1;

    int RobotConstants::motor_drive_frontleft = 4;
    int RobotConstants::motor_drive_backleft = 3;
    int RobotConstants::motor_drive_frontright = 6;
    int RobotConstants::motor_drive_backright = 5;
    int RobotConstants::motor_intake_left = 20;
    int RobotConstants::motor_intake_right = 16;
    int RobotConstants::motor_tray = 8;
    int RobotConstants::motor_lift = 14;

    double RobotConstants::MAX_TRAY_RPM = 60;
    double RobotConstants::MAX_LIFT_RPM = 150;

    double RobotConstants::LIFT_STAGE[2] = {-10, -900};
    double RobotConstants::TRAY_LIFT[4] = {100, 300, 600, 750};
    double RobotConstants::LIFT_PRESETS[4] = {0, -600, -850, -1500};
    double RobotConstants::trayErrorBeforeLiftStart = 25;

    double RobotConstants::TRAY_SCORE = 1050;
    double RobotConstants::SCORE_START_INTAKE = 700;
    double RobotConstants::SCORE_END_INTAKE = 900;
}
