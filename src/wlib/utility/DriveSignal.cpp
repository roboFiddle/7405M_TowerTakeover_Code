#include "DriveSignal.hpp"
#include <string>
#include <sstream>


namespace lib::util {
  DriveSignal::DriveSignal(double l, double r) {
    left_voltage_ = l;
    right_voltage_ = r;
  }
  double DriveSignal::left_voltage() {
    return left_voltage_;
  }
  double DriveSignal::right_voltage() {
    return right_voltage_;
  }
  std::string DriveSignal::toCSV() {
    std::ostringstream stringStream;
    stringStream << "DriveSignal,";
    stringStream << std::to_string(left_voltage_);
    stringStream << ",";
    stringStream << std::to_string(right_voltage_);
    return stringStream.str();
  }
}
