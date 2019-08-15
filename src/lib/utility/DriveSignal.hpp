#ifndef WLIB_UTIL_DRIVESIGNAL_H
#define WLIB_UTIL_DRIVESIGNAL_H

#include "interfaces/CSVLoggable.hpp"
#include "Units.hpp"

namespace util {
  class DriveSignal : public CSVLoggable {
    private:
      double left_voltage_;
      double right_voltage_;
    public:
      DriveSignal(units::Number l, units::Number r);
      double left_voltage();
      double right_voltage();
      std::string toCSV();

      static DriveSignal NEUTRAL;
  };
}

#endif
