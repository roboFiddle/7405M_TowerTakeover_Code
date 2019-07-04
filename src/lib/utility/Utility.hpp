#ifndef WLIB_UTIL_UTILITY
#define WLIB_UTIL_UTILITY

#define EPSILON 0.0001
#define INTERPOLATE(a, b, x) (a + (b-a) * x)
#define LIMIT(value, lower, upper) (value < lower ? lower : ((value > upper) ? upper : value))
#define FEQUALS(a, b) ((a - EPSILON <= b) && (a + EPSILON >= b))

#endif