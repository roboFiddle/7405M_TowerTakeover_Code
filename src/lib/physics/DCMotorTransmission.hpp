//
// Created by alexweiss on 7/4/19.
//

#ifndef INC_7405M_CODE_DCMOTOR_HPP
#define INC_7405M_CODE_DCMOTOR_HPP

#include "../utility/Units.hpp"

namespace physics {
    class DCMotorTransmission {
    protected:
        const units::QAngularSpeed speed_per_volt_; // RADIANS / SECOND
        const units::QTorque torque_per_volt_; // Nm / VOLT
        const double friction_voltage_; // voltage to overcome friction (V)
    public:
        DCMotorTransmission(units::QAngularSpeed speed_per_volt, units::QTorque torque_per_volt, double friction_voltage);
        units::QAngularSpeed speed_per_volt();
        units::QTorque torque_per_volt();
        double friction_voltage();
        units::QAngularSpeed free_speed_at_voltage(double voltage);
        units::QTorque get_torque_at_voltage(units::QAngularSpeed speed, double voltage);
        double get_voltage_for_torque(units::QAngularSpeed speed, units::QTorque torque);

    };
}


#endif //INC_7405M_CODE_DCMOTOR_HPP
