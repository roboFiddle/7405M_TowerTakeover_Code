#ifndef WLIB_UTIL_CSVLOGGABLE_H
#define WLIB_UTIL_CSVLOGGABLE_H

#include <string>

namespace lib::util {
  class CSVLoggable {
    public:
      virtual std::string toCSV() = 0;
  };
}

#endif
