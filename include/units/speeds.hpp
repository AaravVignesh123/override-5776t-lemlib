#pragma once

#include <ratio>
#include <cmath>
#include "quantity.hpp"
#include "lengths.hpp"
#include "time.hpp"

// Max power bot speed in inches per second
#define MAX_SPEED_INCHES_PER_SECOND 18.0  // TODO: find the actual value

// Base speed Quantity (Length Unit / Time Unit)
QUANTITY_TYPE(1, -1, 0, Speed)

// Various length units and corresponding conversions to radians
constexpr Speed motorPowerScale(1.0); // LemLib standard unit (-127 to 127)
constexpr Speed motorPowerFactor = 127 * motorPowerScale;
constexpr Speed inchesPerSecond = MAX_SPEED_INCHES_PER_SECOND * motorPowerFactor;
constexpr Speed feetPerSecond = foot / inch * inchesPerSecond;
constexpr Speed yardsPerSecond = yard / inch * inchesPerSecond;
constexpr Speed tilesPerSecond = tile / inch * inchesPerSecond;
constexpr Speed centimetersPerSecond = centimeter / inch * inchesPerSecond;
constexpr Speed metersPerSecond = meter / centimeter * centimetersPerSecond;

// C++ literals for defining Angles
inline namespace literals {
    constexpr Speed operator"" _power(long double x) {
        return static_cast<double>(x) * motorPowerScale;
    }
    constexpr Speed operator"" _x(long double x) {
        return static_cast<double>(x) * motorPowerFactor;
    }
    constexpr Speed operator"" _in_s(long double x) {
        return static_cast<double>(x) * inchesPerSecond;
    }
    constexpr Speed operator"" _ft_s(long double x) {
        return static_cast<double>(x) * feetPerSecond;
    }
    constexpr Speed operator"" _yd_s(long double x) {
        return static_cast<double>(x) * yardsPerSecond;
    }
    constexpr Speed operator"" _tile_s(long double x) {
        return static_cast<double>(x) * tilesPerSecond;
    }
    constexpr Speed operator"" _cm_s(long double x) {
        return static_cast<double>(x) * centimetersPerSecond;
    }
    constexpr Speed operator"" _m_s(long double x) {
        return static_cast<double>(x) * metersPerSecond;
    }
    constexpr Speed operator"" _power(unsigned long long int x) {
        return static_cast<double>(x) * motorPowerScale;
    }
    constexpr Speed operator"" _x(unsigned long long int x) {
        return static_cast<double>(x) * motorPowerFactor;
    }
    constexpr Speed operator"" _in_s(unsigned long long int x) {
        return static_cast<double>(x) * inchesPerSecond;
    }
    constexpr Speed operator"" _ft_s(unsigned long long int x) {
        return static_cast<double>(x) * feetPerSecond;
    }
    constexpr Speed operator"" _yd_s(unsigned long long int x) {
        return static_cast<double>(x) * yardsPerSecond;
    }
    constexpr Speed operator"" _tile_s(unsigned long long int x) {
        return static_cast<double>(x) * tilesPerSecond;
    }
    constexpr Speed operator"" _cm_s(unsigned long long int x) {
        return static_cast<double>(x) * centimetersPerSecond;
    }
    constexpr Speed operator"" _m_s(unsigned long long int x) {
        return static_cast<double>(x) * metersPerSecond;
    }
}
