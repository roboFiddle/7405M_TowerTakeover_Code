//
// Created by alexweiss on 7/4/19.
//

#ifndef INC_7405M_CODE_DCMOTOR_HPP
#define INC_7405M_CODE_DCMOTOR_HPP

namespace physics {
    class DCMotorTransmission {
    protected:
        const double speed_per_volt_; // RADIANS / SECOND
        const double torque_per_volt_; // Nm / VOLT
        const double friction_voltage_; // voltage to overcome friction (V)
    public:
        DCMotorTransmission(double speed_per_volt, double torque_per_volt, double friction_voltage);
        double speed_per_volt();
        double torque_per_volt();
        double friction_voltage();
        double free_speed_at_voltage(double voltage);
        double get_torque_at_voltage(double speed, double voltage);
        double get_voltage_for_torque(double speed, double torque);

    };
}


#endif //INC_7405M_CODE_DCMOTOR_HPP
