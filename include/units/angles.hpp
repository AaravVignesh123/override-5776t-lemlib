#pragma once

#include <ratio>
#include <cmath>
#include "quantity.hpp"

// Base length Quantity (1 angle unit and 0 other units)
QUANTITY_TYPE(0, 0, 1, Angle)

// Various length units and corresponding conversions to radians
constexpr Angle radian(1.0); // LemLib standard unit
constexpr Angle degree = radian / 180 * M_PI;

// C++ literals for defining Angles
inline namespace literals {
    constexpr Angle operator"" _rad(long double x) {
        return static_cast<double>(x) * radian;
    }
    constexpr Angle operator"" _deg(long double x) {
        return static_cast<double>(x) * degree;
    }
    constexpr Angle operator"" _rad(unsigned long long int x) {
        return static_cast<double>(x) * radian;
    }
    constexpr Angle operator"" _deg(unsigned long long int x) {
        return static_cast<double>(x) * degree;
    }
}
