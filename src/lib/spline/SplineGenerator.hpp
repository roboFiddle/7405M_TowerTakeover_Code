//
// Created by alexweiss on 7/4/19.
//

#ifndef INC_7405M_CODE_SPLINEGENERATOR_HPP
#define INC_7405M_CODE_SPLINEGENERATOR_HPP

#include "Spline.hpp"
#include "../geometry/Pose2dWithCurvature.hpp"
#include <vector>

namespace spline {
    class SplineGenerator {
    private:
        static constexpr double kMaxDX = 2.0;
        static constexpr double kMaxDY = .05;
        static constexpr double kMaxDTheta = 0.05; // RADIANS!!
        static constexpr int kMinSampleSize = 1;
    public:
        static std::vector<geometry::Pose2dWithCurvature> parameterizeSpline(Spline *s, double maxDx, double maxDy,
                double maxDTheta, double t0, double t1);

        static std::vector<geometry::Pose2dWithCurvature> parameterizeSpline(Spline *s);

        static std::vector<geometry::Pose2dWithCurvature>  parameterizeSpline(Spline *s, double maxDx, double maxDy,
                double maxDTheta);

        static std::vector<geometry::Pose2dWithCurvature> parameterizeSplines(std::vector<Spline> splines);

        template <class T>
        static std::vector<geometry::Pose2dWithCurvature> parameterizeSplines(std::vector<T> splines, double maxDx,
                double maxDy, double maxDTheta);

        static void getSegmentArc(Spline *s, std::vector<geometry::Pose2dWithCurvature> *rv, double t0, double t1, double maxDx,
                double maxDy, double maxDTheta);

    };
}

#endif //INC_7405M_CODE_SPLINEGENERATOR_HPP
