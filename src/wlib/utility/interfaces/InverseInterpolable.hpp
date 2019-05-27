#ifndef WLIB_UTIL_REVERSEINTERPOLABLE_H
#define WLIB_UTIL_REVERSEINTERPOLABLE_H

#include <string>

namespace lib::util {
  template<typename T> class InverseInterpolable {
    public:
      virtual double InverseInterpolate(T upper, T query);
  };
}

#endif
