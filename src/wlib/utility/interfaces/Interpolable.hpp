#ifndef WLIB_UTIL_INTERPOLABLE_H
#define WLIB_UTIL_INTERPOLABLE_H

#include <string>

namespace lib::util {
  template<typename T> class Interpolable {
    public:
      virtual T Interpolate(T other, double scale);
  };
}

#endif
