#include "Displacement1d.hpp"
#include "../utility/Utility.hpp"
#include <cmath>
#include <string>
#include <sstream>

namespace lib::geometry {
  Displacement1d::Displacement1d(double displacement) {
    displacement_ = displacement;
  }
  double Displacement1d::distance(Displacement1d other) {
    return std::fabs(displacement_ - other.displacement_);
  }
  bool Displacement1d::operator==(Displacement1d other) {
    return displacement_ == other.displacement_;
  };
  Displacement1d Displacement1d::interpolate(Displacement1d other, double x) {
    return Displacement1d(INTERPOLATE(displacement_, other.displacement_, x));
  }
  std::string Displacement1d::toCSV() {
    std::ostringstream stringStream;
    stringStream << "Displacement1d,";
    stringStream << std::to_string(displacement_);
    return stringStream.str();
  }
  std::string Displacement1d::toString() {
    return toCSV();
  }
}
