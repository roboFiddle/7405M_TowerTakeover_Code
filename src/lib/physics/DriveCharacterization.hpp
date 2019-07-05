//
// Created by alexweiss on 7/4/19.
//

#ifndef INC_7405M_CODE_DRIVECHARACTERIZATION_HPP
#define INC_7405M_CODE_DRIVECHARACTERIZATION_HPP

#include <vector>

namespace physics {
    class DriveCharacterization {
    public:
        class CharacterizationConstants {
        public:
            double ks; //voltage needed to break static friction
            double kv;
            double ka;
        };
        class VelocityDataPoint {
        public:
            const double velocity;
            const double power;
            VelocityDataPoint(double velocity, double power);
        }
        class AccelerationDataPoint {
        public:
            const double velocity;
            const double power;
            const double acceleration;
            AccelerationDataPoint(double velocity, double power, double acceleration);
        };
        class CurvatureDataPoint {
        public:
            const double linear_velocity;
            const double angular_velocity;
            const double left_voltage;
            const double right_voltage;
            CurvatureDataPoint(double linear_velocity, double angular_velocity, double left_voltage,
                    double right_voltage);
        };

        static CharacterizationConstants characterizeDrive(std::vector<VelocityDataPoint> velocityData, std::vector<AccelerationDataPoint> accelerationData);
        static CharacterizationConstants getVelocityCharacterization(double[][] points);
        static CharacterizationConstants getAccelerationCharacterization(double[][] points, CharacterizationConstants velocityCharacterization);

        // Cleans Data
        static double[][] getVelocityData(std::vector<VelocityDataPoint> input)

        static double[][] getAccelerationData(std::vector<AccelerationDataPoint> input, CharacterizationConstants constants)


    };
}


#endif //INC_7405M_CODE_DRIVECHARACTERIZATION_HPP
