//
// Created by alexweiss on 7/3/19.
//

#include "testBase.hpp"
#include <cmath>
#include <stdio.h>

namespace test {
    void assertEquals(double val, double compare, double eps) {
        if(std::fabs(val - compare) > eps) { // BAD
            printf("FAILURE\n");
        }
        else {
            printf("PASS\n");
        }
    }
    void assertTrue(bool val) {
        if(!val) { // BAD
            printf("FAILURE\n");
        }
        else {
            printf("PASS\n");
        }
    }
    void assertFalse(bool val) {
        if(val) { // BAD
            printf("FAILURE\n");
        }
        else {
            printf("PASS\n");
        }
    }
}