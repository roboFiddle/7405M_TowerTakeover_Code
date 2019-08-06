//
// Created by alexweiss on 7/3/19.
//

#ifndef INC_7405M_CODE_TESTBASE_HPP
#define INC_7405M_CODE_TESTBASE_HPP

#include "../lib/utility/Utility.hpp"
#include "../lib/geometry/Translation2d.hpp"

namespace test {
    void assertEquals(geometry::Translation2d  val, geometry::Translation2d compare);
    void assertEquals(double val, double compare);
    void assertEquals(double val, double compare, double eps);
    void assertTrue(bool val);
    void assertFalse(bool val);
}

#endif