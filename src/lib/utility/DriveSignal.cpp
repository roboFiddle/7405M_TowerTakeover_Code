#include "DriveSignal.hpp"
#include <string>
#include <sstream>


namespace util {
  DriveSignal::DriveSignal(units::Number l, units::Number r) {
    left_voltage_ = l.getValue();
    right_voltage_ = r.getValue();
  }
  double DriveSignal::left_voltage() {
    return left_voltage_;
  }
  double DriveSignal::right_voltage() {
    return right_voltage_;
  }
  std::string DriveSignal::toCSV() {
    std::ostringstream stringStream;
    stringStream << "DriveSignal," << std::to_string(left_voltage_) << "," << std::to_string(right_voltage_);
    return stringStream.str();
  }

  DriveSignal DriveSignal::NEUTRAL(0, 0);
}
