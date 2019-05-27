#ifndef WLIB_UTIL_DRIVESIGNAL_H
#define WLIB_UTIL_DRIVESIGNAL_H

#include "interfaces/CSVLoggable.hpp"

namespace lib::util {
  class DriveSignal : public CSVLoggable {
    private:
      double left_voltage_;
      double right_voltage_;
    public:
      DriveSignal(double l, double r);
      double left_voltage();
      double right_voltage();
      std::string toCSV();
  };
}

#endif
