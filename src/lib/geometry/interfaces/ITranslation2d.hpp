#ifndef CODE_V1_ITRANSLATION2D_HPP
#define CODE_V1_ITRANSLATION2D_HPP

#include "State.hpp"

namespace geometry {
  template<typename T> class ITranslation2d : State<T> {
    virtual T translation() = 0;
    virtual double x() = 0;
    virtual double y() = 0;
  };
}

#endif //CODE_V1_ITRANSLATION2D_HPP
