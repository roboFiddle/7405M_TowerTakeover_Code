//
// Created by alexweiss on 7/4/19.
//

#ifndef INC_7405M_CODE_QUINTICHERMITESPLINE_HPP
#define INC_7405M_CODE_QUINTICHERMITESPLINE_HPP

#include "Spline.hpp"
#include "../geometry/Translation2d.hpp"
#include "../geometry/Pose2d.hpp"
#include <vector>

namespace spline {
    typedef struct ControlPoint {
        double ddx;
        double ddy;
    } ControlPoint;

    class QuinticHermiteSpline : public Spline {
    protected:
        double x0_, y0_, dx0_, dy0_, ddx0_, ddy0_; // starting point
        double x1_, y1_, dx1_, dy1_, ddx1_, ddy1_; // ending point

        double ax_, bx_, cx_, dx_, ex_, fx_; // x polynomial coefficients
        double ay_, by_, cy_, dy_, ey_, fy_; // y polynomial coefficients
    private:
        // Tuning Parameters for Numerical Methods and Optimizing
        static constexpr double kEpsilon = 1.0/10000;
        static constexpr double kStepSize = 1.0;
        static constexpr double kMinDelta = 0.001;
        static constexpr int kSamples = 100; // FOR INTEGRATION
        static constexpr int kMaxIterations = 100;

        QuinticHermiteSpline(double x0, double x1, double dx0, double dx1, double ddx0, double ddx1,
                             double y0, double y1, double dy0, double dy1, double ddy0, double ddy1);

        void computeCoefficients();
        double x(double t);
        double dx(double t);
        double ddx(double t);
        double dddx(double t);

        double y(double t);
        double dy(double t);
        double ddy(double t);
        double dddy(double t);
        double sumDCurvature2(); // Returns integral of dCurve^2 over the spline

        static void runOptimizationIteration(std::vector<QuinticHermiteSpline> *splines);

        // Returns X-Cord of Vertex
        static double fitParabola(geometry::Translation2d p1, geometry::Translation2d p2, geometry::Translation2d p3);


    public:
        QuinticHermiteSpline(geometry::Pose2d p0, geometry::Pose2d p1);
        geometry::Pose2d getStartPose();
        geometry::Pose2d getEndPose();
        geometry::Translation2d getPoint(double t);
        double getVelocity(double t);
        double getCurvature(double t);
        double getDCurvature(double t);
        double dCurvature2(double t);
        geometry::Rotation2d getHeading(double t);

        static double sumDCurvature2(std::vector<QuinticHermiteSpline> splines);
        static double optimizeSpline(std::vector<QuinticHermiteSpline> *splines);




    };
}

#endif //INC_7405M_CODE_QUINTICHERMITESPLINE_HPP
