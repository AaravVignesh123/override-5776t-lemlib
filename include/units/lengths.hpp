#pragma once

#include <ratio>
#include <cmath>
#include "quantity.hpp"

// Base length Quantity (1 length unit and 0 other units)
QUANTITY_TYPE(1, 0, 0, Length)

// Various length units and corresponding conversions to inches
constexpr Length inch(1.0); // LemLib standard unit
constexpr Length foot = 12 * inch;
constexpr Length yard = 3 * foot;
constexpr Length tile = 24 * inch;
constexpr Length centimeter = inch / 2.54;
constexpr Length meter = centimeter * 100;
constexpr Length millimeter = meter / 1000;
constexpr Length hole = 0.5 * inch;
constexpr Length segment = 5 * hole;

// C++ literals for defining Lengths
inline namespace literals {
    constexpr Length operator"" _mm(long double x) {
        return static_cast<double>(x) * millimeter;
    }
    constexpr Length operator"" _cm(long double x) {
        return static_cast<double>(x) * centimeter;
    }
    constexpr Length operator"" _m(long double x) {
        return static_cast<double>(x) * meter;
    }
    constexpr Length operator"" _yd(long double x) {
        return static_cast<double>(x) * yard;
    }
    constexpr Length operator"" _ft(long double x) {
        return static_cast<double>(x) * foot;
    }
    constexpr Length operator"" _in(long double x) {
        return static_cast<double>(x) * inch;
    }
    constexpr Length operator"" _tile(long double x) {
        return static_cast<double>(x) * tile;
    }
    constexpr Length operator"" _holes(long double x) {
        return static_cast<double>(x) * hole;
    }
    constexpr Length operator"" _segs(long double x) {
        return static_cast<double>(x) * segment;
    }
    constexpr Length operator"" _mm(unsigned long long int x) {
        return static_cast<double>(x) * millimeter;
    }
    constexpr Length operator"" _cm(unsigned long long int x) {
        return static_cast<double>(x) * centimeter;
    }
    constexpr Length operator"" _m(unsigned long long int x) {
        return static_cast<double>(x) * meter;
    }
    constexpr Length operator"" _yd(unsigned long long int x) {
        return static_cast<double>(x) * yard;
    }
    constexpr Length operator"" _ft(unsigned long long int x) {
        return static_cast<double>(x) * foot;
    }
    constexpr Length operator"" _in(unsigned long long int x) {
        return static_cast<double>(x) * inch;
    }
    constexpr Length operator"" _tile(unsigned long long int x) {
        return static_cast<double>(x) * tile;
    }
    constexpr Length operator"" _holes(unsigned long long int x) {
        return static_cast<double>(x) * hole;
    }
    constexpr Length operator"" _segs(unsigned long long int x) {
        return static_cast<double>(x) * segment;
    }
}
