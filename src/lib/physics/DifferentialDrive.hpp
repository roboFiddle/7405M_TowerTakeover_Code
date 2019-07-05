//
// Created by alexweiss on 7/4/19.
//

#ifndef INC_7405M_CODE_DIFFERENTIALDRIVE_HPP
#define INC_7405M_CODE_DIFFERENTIALDRIVE_HPP

#include "DCMotorTransmission.hpp"
#include <string>

namespace physics {
    class DifferentialDrive {
    protected:
        const double mass_; // Kg
        const double moi_; // moment of intertia - KgM^2
        const double angular_drag_; // Nm/rad/s
        const double wheel_radius_; //meters
        const double effective_wheelbase_radius_; //m

        DCMotorTransmission left_;
        DCMotorTransmission right_;

    public:
        class MinMax {
        public:
            double min_;
            double max_;
        };

        class ChassisState {
        public:
            double linear_;
            double angular_;
            ChassisState(double linear, double angular);
            ChassisState();
            std::string toString();
        };

        class WheelState {
        public:
            double left_;
            double right_;
            WheelState(double left, double right);
            WheelState();
            double get(bool get_left);
            void set(bool set_left, double val);
            std::string toString();
        };

        class DriveDynamics {
        public:
            double curvature = 0.0;  // m^-1
            double dcurvature = 0.0;  // m^-1/m
            ChassisState chassis_velocity = new ChassisState();  // m/s
            ChassisState chassis_acceleration = new ChassisState();  // m/s^2
            WheelState wheel_velocity = new WheelState();  // rad/s
            WheelState wheel_acceleration = new WheelState();  // rad/s^2
            WheelState voltage = new WheelState();  // V
            WheelState wheel_torque = new WheelState(); // N m
            std::string toCSV();
            std::string toString();
        };

        DifferentialDrive(double mass, double moi, double angular_drag,
                double wheel_radius, double effective_wheelbase_radius,
                DCMotorTransmission left_transmission, DCMotorTransmission right_transmission);
        double mass();
        double moi();
        double wheel_radius();
        double effective_wheelbase_radius();
        DCMotorTransmission* left_transmission();
        DCMotorTransmission* right_transmission();
        ChassisState solveForwardKinematics( WheelState wheel_motion);
        WheelState solveInverseKinematics(ChassisState chassis_motion);

        // Solve for torques and accelerations.
        DriveDynamics solveForwardDynamics(ChassisState chassis_velocity, WheelState voltage);
        DriveDynamics solveForwardDynamics(WheelState wheel_velocity, WheelState voltage);

        // Assumptions about dynamics: velocities and voltages provided.
        void solveForwardDynamics(DriveDynamics* dynamics);

        // Solve for torque and voltage
        DriveDynamics solveInverseDynamics(ChassisState chassis_velocity, ChassisState chassis_acceleration);
        DriveDynamics solveInverseDynamics(WheelState wheel_velocity, WheelState wheel_acceleration);

        // Assumptions about dynamics: velocities and accelerations provided, curvature and dcurvature computed.
        void solveInverseDynamics(DriveDynamics dynamics);
        double getMaxAbsVelocity(double curvature, double max_abs_voltage);

        // Curvature is redundant here in the case that chassis_velocity is not purely angular.  It is the responsibility of
        // the caller to ensure that curvature = angular vel / linear vel in these cases.
        MinMax getMinMaxAcceleration(final ChassisState chassis_velocity, double curvature, double max_abs_voltage);
    };
}


#endif //INC_7405M_CODE_DIFFERENTIALDRIVE_HPP
