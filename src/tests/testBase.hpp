//
// Created by alexweiss on 7/3/19.
//

#ifndef INC_7405M_CODE_TESTBASE_HPP
#define INC_7405M_CODE_TESTBASE_HPP

#define EPSILON .001

namespace test {
    void assertEquals(double val, double compare, double eps);
    void assertTrue(bool val);
    void assertFalse(bool val);
}

#endif