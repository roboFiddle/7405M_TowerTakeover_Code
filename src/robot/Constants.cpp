#include "Constants.hpp"

namespace constants {
    units::QLength RobotConstants::kDriveWheelRadius = 2*units::inch;
    units::QMass RobotConstants::kRobotMass = 4*units::kg;
    units::QMoment RobotConstants::kRobotMoment = 6*units::kgm2;
    units::QAngularDrag RobotConstants::kRobotAngularDrag = 0.0;
    units::QLength RobotConstants::kDriveWheelTrackWidth = 14.875 * units::inch;
    units::Number RobotConstants::kTrackScrubFactor = 1.0;

    units::QLength RobotConstants::kDeadwheelRadius = 1.375*units::inch;
    units::QLength RobotConstants::kDeadwheelBaseWidth = 3.56*units::inch;
    units::QLength RobotConstants::kDeadwheelBackTurningRadius = 9.45 * units::inch;

    units::QAngularSpeed RobotConstants::kDriveSpeedPerVolt = 1.09; // RADIANS / SECOND / VOTL
    units::QTorque RobotConstants::kDriveTorquePerVolt = 0.56; // Nm / VOLT
    units::Number RobotConstants::kDriveFrictionVoltage = 1.0; // voltage to overcome friction (V)

    double RobotConstants::turnKP = 50;
    double RobotConstants::turnKI = 0;
    double RobotConstants::turnKD = 5;

    int RobotConstants::motor_drive_frontleft = 9;
    int RobotConstants::motor_drive_backleft = 10;
    int RobotConstants::motor_drive_frontright = 2;
    int RobotConstants::motor_drive_backright = 5;
    int RobotConstants::motor_intake_left = 8;
    int RobotConstants::motor_intake_right = 13;
    int RobotConstants::motor_tray = 3;
    int RobotConstants::motor_lift = 18;

    double RobotConstants::MAX_TRAY_RPM = 100;
    double RobotConstants::MAX_LIFT_RPM = 200;

    double RobotConstants::LIFT_STAGE[1] = {500};
    double RobotConstants::TRAY_LIFT[2] = {75, 1800};
    double RobotConstants::LIFT_PRESETS[3] = {340, 2200, 3200};
    double RobotConstants::trayErrorBeforeLiftStart = 1000;
    double RobotConstants::liftErrorBeforeTrayStart = 100;

    double RobotConstants::TRAY_SCORE = 3200;
    double RobotConstants::SCORE_START_INTAKE = 1200;
    double RobotConstants::SCORE_END_INTAKE = 1800;
}
