#ifndef WLIB_UTIL_STATE_H
#define WLIB_UTIL_STATE_H

#include "../../utility/interfaces/CSVLoggable.hpp"
#include "../../utility/interfaces/Interpolable.hpp"

namespace lib::geometry {
  template<typename T> class State : lib::util::CSVLoggable, lib::util::Interpolable<T> {
    public:
      virtual double distance(T other) = 0;
      virtual bool operator==(T other) = 0;
      virtual std::string toCSV() = 0;
      virtual std::string toString() = 0;
  };
}


#endif
