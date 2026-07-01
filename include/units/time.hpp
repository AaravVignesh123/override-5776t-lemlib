#pragma once

#include <ratio>
#include <cmath>
#include "quantity.hpp"

// Base time Quantity (1 time unit and 0 other units)
QUANTITY_TYPE(0, 1, 0, Time)

// Various time units and corresponding conversions to seconds
constexpr Time millisecond(1.0); // LemLib standard unit
constexpr Time second = millisecond * 1000;
constexpr Time microsecond = millisecond / 1000;
constexpr Time minute = 60 * second;
constexpr Time hour = 60 * minute;

// C++ literals for defining Times
inline namespace literals {
    constexpr Time operator"" _s(long double x) {
        return static_cast<double>(x) * second;
    }
    constexpr Time operator"" _ms(long double x) {
        return static_cast<double>(x) * millisecond;
    }
    constexpr Time operator"" _Ms(long double x) {
        return static_cast<double>(x) * microsecond;
    }
    constexpr Time operator"" _min(long double x) {
        return static_cast<double>(x) * minute;
    }
    constexpr Time operator"" _hr(long double x) {
        return static_cast<double>(x) * hour;
    }
    constexpr Time operator"" _s(unsigned long long int x) {
        return static_cast<double>(x) * second;
    }
    constexpr Time operator"" _ms(unsigned long long int x) {
        return static_cast<double>(x) * millisecond;
    }
    constexpr Time operator"" _Ms(unsigned long long int x) {
        return static_cast<double>(x) * microsecond;
    }
    constexpr Time operator"" _min(unsigned long long int x) {
        return static_cast<double>(x) * minute;
    }
    constexpr Time operator"" _hr(unsigned long long int x) {
        return static_cast<double>(x) * hour;
    }
}
