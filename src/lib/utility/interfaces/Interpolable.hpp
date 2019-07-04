#ifndef WLIB_UTIL_INTERPOLABLE_H
#define WLIB_UTIL_INTERPOLABLE_H

#include <string>

namespace util {
  template<typename T> class Interpolable {
    public:
      virtual T interpolate(T other, double scale);
  };
}

#endif
