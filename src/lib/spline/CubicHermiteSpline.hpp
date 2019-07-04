//
// Created by alexweiss on 7/3/19.
//

#ifndef INC_7405M_CODE_CUBICHERMITESPLINE_HPP
#define INC_7405M_CODE_CUBICHERMITESPLINE_HPP

#include "Spline.hpp"
#include "../geometry/Translation2d.hpp"
#include "../geometry/Rotation2d.hpp"
#include "../geometry/Pose2d.hpp"


namespace spline {
    class CubicHermiteSpline {
    public:
        double ax_, bx_, cx_, dx_;
        double ay_, by_, cy_, dy_;
        double x(double t);
        double y(double t);
        double dx(double t);
        double dy(double t);
        double ddx(double t);
        double ddy(double t);
        double dddx(double t);
        double dddy(double t);


        CubicHermiteSpline(geometry::Pose2d p0, geometry::Pose2d p1);
        geometry::Translation2d getPoint(double t);
        geometry::Rotation2d getHeading(double t);
        double getVelocity(double t);
        double getCurvature(double t);
        double getDCurvature(double t);
    };
}


#endif //INC_7405M_CODE_CUBICHERMITESPLINE_HPP
