//
// Created by alexweiss on 7/4/19.
//

#include "SplineGenerator.hpp"
#include "Spline.hpp"
#include "../geometry/Pose2dWithCurvature.hpp"
#include <vector>

namespace spline {
    std::vector<geometry::Pose2dWithCurvature> SplineGenerator::parameterizeSpline(Spline *s, double maxDx, double maxDy, double maxDTheta, double t0, double t1) {
        std::vector<geometry::Pose2dWithCurvature> rv;
        rv.push_back(s->getPose2dWithCurvature(0.0));
        double dt = (t1 - t0);
        for (double t = 0; t < t1; t += dt / kMinSampleSize) {
            getSegmentArc(s, &rv, t, t + dt / kMinSampleSize, maxDx, maxDy, maxDTheta);
        }
        return rv;
    }
    std::vector<geometry::Pose2dWithCurvature> SplineGeneratorparameterizeSpline(Spline *s) {

    }
    std::vector<geometry::Pose2dWithCurvature>  SplineGeneratorparameterizeSpline(Spline *s, double maxDx, double maxDy, double maxDTheta) {

    }
    std::vector<geometry::Pose2dWithCurvature> SplineGeneratorparameterizeSplines(std::vector<Spline> splines) {

    }

    template <class T>
    std::vector<geometry::Pose2dWithCurvature> SplineGeneratorparameterizeSplines(std::vector<T> splines, double maxDx, double maxDy, double maxDTheta) {
        static_assert(std::is_base_of<Spline, T>::value, "T must derive from Spline");

    }

    void SplineGeneratorgetSegmentArc(Spline *s, std::vector<geometry::Pose2dWithCurvature> *rv, double t0, double t1, double maxDx, double maxDy, double maxDTheta) {
    }
}