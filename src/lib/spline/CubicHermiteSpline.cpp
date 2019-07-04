//
// Created by alexweiss on 7/3/19.
//

#include "CubicHermiteSpline.hpp"
#include "../geometry/Translation2d.hpp"
#include "../geometry/Rotation2d.hpp"
#include "../geometry/Pose2d.hpp"
#include <cmath>

namespace spline{
    CubicHermiteSpline::CubicHermiteSpline(geometry::Pose2d p0, geometry::Pose2d p1) {
        double x0, x1, dx0, dx1, y0, y1, dy0, dy1;
        double scale = 2 * p0.translation().distance(p1.translation());
        x0 = p0.translation().x();
        x1 = p1.translation().x();
        dx0 = p0.rotation().cos() * scale;
        dx1 = p1.rotation().cos() * scale;
        y0 = p0.translation().y();
        y1 = p1.translation().y();
        dy0 = p0.rotation().sin() * scale;
        dy1 = p1.rotation().sin() * scale;
        ax_ = dx0 + dx1 + 2 * x0 - 2 * x1;
        bx_ = -2 * dx0 - dx1 - 3 * x0 + 3 * x1;
        cx_ = dx0;
        dx_ = x0;
        ay_ = dy0 + dy1 + 2 * y0 - 2 * y1;
        by_ = -2 * dy0 - dy1 - 3 * y0 + 3 * y1;
        cy_ = dy0;
        dy_ = y0;
    }
    geometry::Translation2d CubicHermiteSpline::getPoint(double t) {
        return geometry::Translation2d(x(t), y(t));
    }
    geometry::Rotation2d CubicHermiteSpline::getHeading(double t) {
        return geometry::Rotation2d(x(t), y(t));
    }
    double CubicHermiteSpline::getVelocity(double t) {
        return std::hypot(dx(t), dy(t));
    }
    double CubicHermiteSpline::getCurvature(double t) {
        double a = (dx(t) * ddy(t) - dy(t) * ddx(t));
        double b = (dx(t) * dx(t) + dy(t) * dy(t)) * std::sqrt(dx(t) * dx(t) + dy(t) * dy(t));
        return a / b;
    }
    double CubicHermiteSpline::getDCurvature(double t) {
        double dx2dy2 = (dx(t) * dx(t) + dy(t) * dy(t));
        double num = (dx(t) * dddy(t) - dddx(t) * dy(t)) * dx2dy2 - 3 * (dx(t) * ddy(t) - ddx(t) * dy(t)) * (dx(t) * ddx(t) + dy(t) * ddy(t));
        return num / (dx2dy2 * dx2dy2 * std::sqrt(dx2dy2));
    }

    double CubicHermiteSpline::x(double t) {
        return t * t * t * ax_ + t * t * bx_ + t * cx_ + dx_;
    }
    double CubicHermiteSpline::y(double t) {
        return t * t * t * ay_ + t * t * by_ + t * cy_ + dy_;
    }
    double CubicHermiteSpline::dx(double t) {
        return 3 * t * t * ax_  + 2 * t * bx_ + cx_;
    }
    double CubicHermiteSpline::dy(double t) {
        return 3 * t * t * ay_  + 2 * t * by_ + cy_;
    }
    double CubicHermiteSpline::ddx(double t) {
        return 6 * t * ax_ + 2 * bx_;
    }
    double CubicHermiteSpline::ddy(double t) {
        return 6 * t * ay_ + 2 * by_;
    }
    double CubicHermiteSpline::dddx(double t) {
        return 6 * ax_;
    }
    double CubicHermiteSpline::dddy(double t) {
        return 6 * ay_;
    }
}