#ifndef WLIB_UTIL_DISPLACEMENT1D_H
#define WLIB_UTIL_DISPLACEMENT1D_H

#include "interfaces/State.hpp"

namespace geometry {
  class Displacement1d : State<Displacement1d> {
    private:
      double displacement_;
    public:
      Displacement1d(double displacement);
      double distance(Displacement1d other);
      bool operator==(Displacement1d other);
      Displacement1d interpolate(Displacement1d other, double x);
      std::string toCSV();
      std::string toString();
  };
}



#endif
