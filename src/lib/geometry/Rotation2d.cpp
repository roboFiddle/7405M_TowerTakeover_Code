#define _USE_MATH_DEFINES

#include "Rotation2d.hpp"
#include "Translation2d.hpp"
#include "../utility/Utility.hpp"
#include <cmath>
#include <math.h>
#include <string>
#include <sstream>

namespace geometry {

    Rotation2d::Rotation2d(Translation2d other) {
      Rotation2d(other.x(), other.y());
    }

    Rotation2d::Rotation2d(double x, double y) {
      double magnitude = std::hypot(x,y);
      sin_angle_ = y / magnitude;
      cos_angle_ = x / magnitude;
    }

    double Rotation2d::cos() {
      return cos_angle_;
    }
    double Rotation2d::sin() {
      return sin_angle_;
    }
    double Rotation2d::tan() {
      if(std::fabs(cos_angle_) < EPSILON) {
          if (sin_angle_ > 0)
              return HUGE_VAL;
          return -HUGE_VAL;
      }
      return sin_angle_ / cos_angle_;
    }
    double Rotation2d::getRadians() {
      return std::atan2(sin_angle_, cos_angle_);
    }
    double Rotation2d::getDegrees() {
      return getRadians() * 180.0 / M_PI;
    }
    Rotation2d Rotation2d::rotateBy(Rotation2d other) {
      return Rotation2d(cos_angle_ * other.cos_angle_ - sin_angle_ * other.sin_angle_,
                            cos_angle_ * other.sin_angle_ + sin_angle_ * other.cos_angle_);
    }
    Rotation2d Rotation2d::normal() {
      return Rotation2d(-sin_angle_, cos_angle_);
    }
    Rotation2d Rotation2d::inverse() {
      return Rotation2d(cos_angle_, -sin_angle_);
    }
    bool Rotation2d::isParallel(Rotation2d other) {
      return FEQUALS(Translation2d::cross(toTranslation(), other.toTranslation()), 0.0);
    }
    Translation2d Rotation2d::toTranslation() {
      return Translation2d(cos_angle_, sin_angle_);
    }
    double Rotation2d::distance(Rotation2d other) {
      return inverse().rotateBy(other).getRadians();
    }
    bool Rotation2d::operator==(Rotation2d other) {
      return FEQUALS(cos_angle_, other.cos_angle_) && FEQUALS(sin_angle_, other.sin_angle_);
    }
    Rotation2d Rotation2d::interpolate(Rotation2d other, double x) {
      x = LIMIT(x, 0.0, 1.0);
      double angle_diff = inverse().rotateBy(other).getRadians();
      return rotateBy(Rotation2d::fromRadians(angle_diff * x));
    }
    std::string Rotation2d::toCSV() {
      std::ostringstream stringStream;
      stringStream << "Rotation2d," << std::to_string(cos_angle_) << "," << std::to_string(sin_angle_);
      return stringStream.str();
    }
    std::string Rotation2d::toString() {
      return toCSV();
    }

    Rotation2d Rotation2d::fromRadians(double angle_radians) {
      return Rotation2d(std::cos(angle_radians), std::sin(angle_radians));
    }
    Rotation2d Rotation2d::fromDegrees(double angle_degrees) {
      return fromRadians(angle_degrees * M_PI / 180.0);
    }
}