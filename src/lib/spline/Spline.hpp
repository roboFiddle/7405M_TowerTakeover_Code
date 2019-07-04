//
// Created by alexweiss on 7/3/19.
//

#ifndef INC_7405M_CODE_SPLINE_HPP
#define INC_7405M_CODE_SPLINE_HPP

#include "../geometry/Translation2d.hpp"
#include "../geometry/Rotation2d.hpp"
#include "../geometry/Pose2d.hpp"
#include "../geometry/Pose2dWithCurvature.hpp"

namespace spline {
    class Spline {
    public:
        virtual geometry::Translation2d getPoint(double t) = 0;
        virtual geometry::Rotation2d getHeading(double t) = 0;
        virtual double getCurvature(double t) = 0;
        virtual double getDCurvature(double t) = 0;
        virtual double getVelocity(double t) = 0;
        geometry::Pose2d getPose2d(double t);
        geometry::Pose2dWithCurvature getPose2dWithCurvature(double t);
    };
}

#endif //INC_7405M_CODE_SPLINE_HPP
