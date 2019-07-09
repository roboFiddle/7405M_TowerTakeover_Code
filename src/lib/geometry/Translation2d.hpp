#ifndef CODE_V1_TRANSLATION2D_HPP
#define CODE_V1_TRANSLATION2D_HPP

#include "interfaces/ITranslation2d.hpp"
#include "Rotation2d.hpp"

namespace geometry {
    class Translation2d : ITranslation2d<Translation2d> {
    private:
        double x_;
        double y_;
    public:
        Translation2d();
        Translation2d(double x, double y);
        Translation2d(Translation2d start, Translation2d end);
        Translation2d translation();
        double norm();
        double x();
        double y();
        Translation2d translateBy(Translation2d other);
        Translation2d rotateBy(Rotation2d rotation);
        Rotation2d direction();
        Translation2d inverse();
        Translation2d interpolate(Translation2d other, double x);
        Translation2d extrapolate(Translation2d other, double x);
        Translation2d scale(double s);
        bool operator==(Translation2d other);
        double distance(Translation2d other);

        std::string toCSV();
        std::string toString();

        static double dot(Translation2d a, Translation2d b);
        static double cross(Translation2d a, Translation2d b);
        static Rotation2d getAngle(Translation2d a, Translation2d b);
    };
}

#endif //CODE_V1_TRANSLATION2D_HPP
