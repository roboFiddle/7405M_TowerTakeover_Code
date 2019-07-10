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
        const units::QMass mass_; // Kg
        const units::QMoment moi_; // moment of intertia - KgM^2
        const units::QAngularDrag angular_drag_; // Nm/rad/s
        const units::QLength wheel_radius_; //meters
        const units::QLength effective_wheelbase_radius_; //m

        DCMotorTransmission left_;
        DCMotorTransmission right_;

    public:
      class MinMaxAcceleration {
       public:
        units::QAcceleration min_acceleration_;
        units::QAcceleration max_acceleration_;
        MinMaxAcceleration();
        MinMaxAcceleration(double min_acceleration, double max_acceleration);
        units::QAcceleration min_acceleration();
        units::QAcceleration max_acceleration();
        bool valid();
        static MinMaxAcceleration kNoLimits ;
      };

        template <typename L, typename A>
        class ChassisState {
        public:
            L linear_;
            A angular_;
            ChassisState<L,A>(L linear, A angular) {
              linear_ = linear;
              angular_ = angular;
            }
            ChassisState<L,A>() {
              linear_ = 0;
              angular_ = 0;
            }
            std::string toString();
        };

        template <typename T>
        class WheelState {
        public:
            T left_;
            T right_;
            WheelState(T left, T right) {
              left_ = left;
              right_ = right;
            }
            WheelState() {
              left_ = 0;
              right_ = 0;
            }
            T get(bool get_left) {
              return (get_left ? left_ : right_);
            }
            void set(bool set_left, T val) {
              if(set_left)
                left_ = val;
              else
                right_ = val;
            }
            std::string toString();
        };

        class DriveDynamics {
        public:
            units::QCurvature curvature = 0.0;  // m^-1
            units::QDCurvature dcurvature = 0.0;  // m^-1/m
            ChassisState<units::QSpeed, units::QAngularSpeed> chassis_velocity;  // m/s
            ChassisState<units::QAcceleration, units::QAngularAcceleration> chassis_acceleration;  // m/s^2
            WheelState<units::QAngularSpeed> wheel_velocity;  // rad/s
            WheelState<units::QAngularAcceleration> wheel_acceleration;  // rad/s^2
            WheelState<double> voltage;  // V
            WheelState<units::QTorque> wheel_torque; // N m
            std::string toCSV();
            std::string toString();
        };

        DifferentialDrive(units::QMass mass, units::QMoment moi, units::QAngularDrag angular_drag,
                          units::QLength wheel_radius, units::QLength effective_wheelbase_radius,
                          DCMotorTransmission left_transmission, DCMotorTransmission right_transmission);
        units::QMass mass();
        units::QMoment moi();
        units::QLength wheel_radius();
        units::QLength effective_wheelbase_radius();
        DCMotorTransmission* left_transmission();
        DCMotorTransmission* right_transmission();
        ChassisState<units::QSpeed, units::QAngularSpeed> solveForwardKinematics(WheelState<units::QAngularSpeed> wheel_motion);
        ChassisState<units::QAcceleration, units::QAngularAcceleration> solveForwardKinematics(WheelState<units::QAngularAcceleration> wheel_motion);
        WheelState<units::QAngularSpeed> solveInverseKinematics(ChassisState<units::QSpeed, units::QAngularSpeed> chassis_motion);
        WheelState<units::QAngularAcceleration> solveInverseKinematics(ChassisState<units::QAcceleration, units::QAngularAcceleration> chassis_motion);

        // Solve for torques and accelerations.
        DriveDynamics solveForwardDynamics(ChassisState<units::QSpeed, units::QAngularSpeed> chassis_velocity, WheelState<double> voltage);
        DriveDynamics solveForwardDynamics(WheelState<units::QAngularSpeed> wheel_velocity, WheelState<double> voltage);

        // Assumptions about dynamics: velocities and voltages provided.
        void solveForwardDynamics(DriveDynamics* dynamics);

        // Solve for torque and voltage
        DriveDynamics solveInverseDynamics(ChassisState<units::QSpeed, units::QAngularSpeed> chassis_velocity, ChassisState<units::QAcceleration, units::QAngularAcceleration> chassis_acceleration);
        DriveDynamics solveInverseDynamics(WheelState<units::QAngularSpeed> wheel_velocity, WheelState<units::QAngularAcceleration> wheel_acceleration);

        // Assumptions about dynamics: velocities and accelerations provided, curvature and dcurvature computed.
        void solveInverseDynamics(DriveDynamics* dynamics);
        units::QSpeed getMaxAbsVelocity(units::QCurvature curvature, double max_abs_voltage);

        // Curvature is redundant here in the case that chassis_velocity is not purely angular.  It is the responsibility of
        // the caller to ensure that curvature = angular vel / linear vel in these cases.
        MinMaxAcceleration getMinMaxAcceleration(ChassisState<units::QSpeed, units::QAngularSpeed> chassis_velocity, units::QCurvature curvature, double max_abs_voltage);
    };
}


#endif //INC_7405M_CODE_DIFFERENTIALDRIVE_HPP
