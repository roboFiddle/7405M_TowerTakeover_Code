#ifndef WLIB_UTIL_INTERPOLABLE_DOUBLE_H
#define WLIB_UTIL_INTERPOLABLE_DOUBLE_H

#include "interfaces/Interpolable.hpp"
#include "interfaces/InverseInterpolable.hpp"

namespace util {
  class InterpolatingDouble : public Interpolable<double>, public InverseInterpolable<double> {
    private:
      double val_;
    public:
      InterpolatingDouble(double v);
      InterpolatingDouble interpolate(InterpolatingDouble other, double x);
      inline double getVal() const { return val_; }
      double inverseInterpolate(InterpolatingDouble upper, InterpolatingDouble query);
      bool operator<(const InterpolatingDouble& other);
      bool operator<(const InterpolatingDouble other);
      bool operator<=(const InterpolatingDouble& other);
      bool operator>(const InterpolatingDouble& other);
      bool operator>=(const InterpolatingDouble& other);
  };

  static bool operator<(InterpolatingDouble const& a, InterpolatingDouble const& b);
}

#endif
