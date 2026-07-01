#pragma once

#include <ratio>
#include <cmath>

// Short-hand for defining new quantity types in terms of exponents on SI units of length, time, and angle
#define QUANTITY_TYPE(_Ldim, _Tdim, _Adim, name) typedef Quantity<std::ratio<_Ldim>, std::ratio<_Tdim>, std::ratio<_Adim>> name;

// Generic Quantity class
template <typename LengthDim, typename TimeDim, typename AngleDim>
class Quantity {
    public:
    explicit constexpr Quantity() : value(0.0) {}

    explicit constexpr Quantity(double val) : value(val) {}

    // Direct modification operators (+=, -=, *=, /=, *-1)
    constexpr Quantity const &operator+=(const Quantity &other) {
        value += other.value;
        return *this;
    }

    constexpr Quantity const &operator-=(const Quantity &other) {
        value -= other.value;
        return *this;
    }

    constexpr Quantity const &operator*=(const double other) {
        value *= other;
        return *this;
    }

    constexpr Quantity const &operator/=(const double other) {
        value /= other;
        return *this;
    }

    constexpr Quantity operator-() {
        return Quantity(value * -1);
    }

    // Returns the value of the Quantity in multiples of the specified unit
    constexpr double convert(const Quantity &other) const {
        return value / other.value;
    }

    // Returns the raw value of the Quantity (internal use only)
    constexpr double getValue() const {
        return value;
    }

    // Returns the sign of the Quantity (-1, 0, 1)
    constexpr double sign() const {
        return value ? (std::signbit(value) ? -1 : 1) : 0;
    }

    // More nuanced operators (abs, sqrt)
    constexpr Quantity<LengthDim, TimeDim, AngleDim> abs() const {
        return Quantity<LengthDim, TimeDim, AngleDim>(std::fabs(value));
    }

    constexpr Quantity<
        std::ratio_divide<LengthDim, std::ratio<2>>,
        std::ratio_divide<TimeDim, std::ratio<2>>,
        std::ratio_divide<AngleDim, std::ratio<2>>>
    sqrt() const {
    return Quantity<
        std::ratio_divide<LengthDim, std::ratio<2>>,
        std::ratio_divide<TimeDim, std::ratio<2>>,
        std::ratio_divide<AngleDim, std::ratio<2>>>(std::sqrt(value));
    }

    private:
    double value;
};


// Standard arithmetic operators between Quantities (+, -, *, /)
template <typename L, typename T, typename A>
constexpr Quantity<L, T, A> operator+(const Quantity<L, T, A> &self,
                                          const Quantity<L, T, A> &other) {
    return Quantity<L, T, A>(self.getValue() + other.getValue());
}

template <typename L, typename T, typename A>
constexpr Quantity<L, T, A> operator-(const Quantity<L, T, A> &self,
                                          const Quantity<L, T, A> &other) {
    return Quantity<L, T, A>(self.getValue() - other.getValue());
}

// Multiply two Quantities by multiplying their direct values and combining the units (as standard)
template <
    typename L1, typename T1, typename A1,
    typename L2, typename T2, typename A2
>
constexpr Quantity<
    std::ratio_add<L1, L2>,
    std::ratio_add<T1, T2>,
    std::ratio_add<A1, A2>
>
operator*(const Quantity<L1, T1, A1> &self, const Quantity<L2, T2, A2> &other) {
    return Quantity<
        std::ratio_add<L1, L2>,
        std::ratio_add<T1, T2>,
        std::ratio_add<A1, A2>
    >(self.getValue() * other.getValue());
}

// Multiply a Quantity and a scalar by scaling the value of the Quantity
template <typename L, typename T, typename A>
constexpr Quantity<L, T, A> operator*(const double &self, const Quantity<L, T, A> &other) {
    return Quantity<L, T, A>(self * other.getValue());
}
template <typename L, typename T, typename A>
constexpr Quantity<L, T, A> operator*(const Quantity<L, T, A> &self, const double &other) {
    return Quantity<L, T, A>(self.getValue() * other);
}

// Divide two Quantities by dividing their direct values and cancelling the units (as standard)
template <
    typename L1, typename T1, typename A1,
    typename L2, typename T2, typename A2
>
constexpr Quantity<
    std::ratio_subtract<L1, L2>,
    std::ratio_subtract<T1, T2>,
    std::ratio_subtract<A1, A2>
>
operator/(const Quantity<L1, T1, A1> &self, const Quantity<L2, T2, A2> &other) {
    return Quantity<
        std::ratio_subtract<L1, L2>,
        std::ratio_subtract<T1, T2>,
        std::ratio_subtract<A1, A2>
    >(self.getValue() / other.getValue());
}

// Divide a scalar by a Quantity by scaling the value of the Quantity and inverting the units
template <typename L, typename T, typename A>
constexpr Quantity<
    std::ratio_subtract<std::ratio<0>, L>,
    std::ratio_subtract<std::ratio<0>, T>,
    std::ratio_subtract<std::ratio<0>, A>
>
operator/(const double &x, const Quantity<L, T, A> &other) {
    return Quantity<
        std::ratio_subtract<std::ratio<0>, L>,
        std::ratio_subtract<std::ratio<0>, T>,
        std::ratio_subtract<std::ratio<0>, A>
    >(x / other.getValue());
}

// Divide a Quantity by a scalar by scaling the value of the Quantity
template <typename L, typename T, typename A>
constexpr Quantity<L, T, A> operator/(const Quantity<L, T, A> &other, const double &x) {
    return Quantity<L, T, A>(other.getValue() / x);
}

// Common trig functions
// Unitless Quantity for trig outputs
QUANTITY_TYPE(0, 0, 0, Unitless)
constexpr Unitless unitless(1.0);

// Sin for Angle Quantities
constexpr Unitless
sin(const Quantity<std::ratio<0>, std::ratio<0>, std::ratio<1>> &rhs) {
    return Unitless(std::sin(rhs.getValue()));
}

// Cos for Angle Quantities
constexpr Unitless
cos(const Quantity<std::ratio<0>, std::ratio<0>, std::ratio<1>> &rhs) {
    return Unitless(std::cos(rhs.getValue()));
}

// Tan for Angle Quantities
constexpr Unitless
tan(const Quantity<std::ratio<0>, std::ratio<0>, std::ratio<1>> &rhs) {
    return Unitless(std::tan(rhs.getValue()));
}

// Arcsin for Angle Quantities
constexpr Quantity<std::ratio<0>, std::ratio<0>, std::ratio<1>>
asin(const Unitless &rhs) {
    return Quantity<std::ratio<0>, std::ratio<0>, std::ratio<1>>(
        std::asin(rhs.getValue()));
}

// Arccos for Angle Quantities
constexpr Quantity<std::ratio<0>, std::ratio<0>, std::ratio<1>>
acos(const Unitless &rhs) {
    return Quantity<std::ratio<0>, std::ratio<0>, std::ratio<1>>(
        std::acos(rhs.getValue()));
}

// Arctan for Angle Quantities
constexpr Quantity<std::ratio<0>, std::ratio<0>, std::ratio<1>>
atan(const Unitless &rhs) {
    return Quantity<std::ratio<0>, std::ratio<0>, std::ratio<1>>(
        std::atan(rhs.getValue()));
}

// Atan2 for Angle Quantities
template <typename L, typename T, typename A>
constexpr Quantity<std::ratio<0>, std::ratio<0>, std::ratio<1>>
atan2(const Quantity<L, T, A> &self, const Quantity<L, T, A> &rhs) {
    return Quantity<std::ratio<0>, std::ratio<0>, std::ratio<1>>(
        std::atan2(self.getValue(), rhs.getValue()));
}

// Comparisons between Quantities (<, <=, ==, =>, >, !=)
template <typename L, typename T, typename A>
constexpr bool operator<(const Quantity<L, T, A> &lhs, const Quantity<L, T, A> &rhs) {
    return lhs.getValue() < rhs.getValue();
}

template <typename L, typename T, typename A>
constexpr bool operator<=(const Quantity<L, T, A> &lhs, const Quantity<L, T, A> &rhs) {
    return lhs.getValue() <= rhs.getValue();
}

template <typename L, typename T, typename A>
constexpr bool operator==(const Quantity<L, T, A> &lhs, const Quantity<L, T, A> &rhs) {
    return lhs.getValue() == rhs.getValue();
}

template <typename L, typename T, typename A>
constexpr bool operator>=(const Quantity<L, T, A> &lhs, const Quantity<L, T, A> &rhs) {
    return lhs.getValue() >= rhs.getValue();
}

template <typename L, typename T, typename A>
constexpr bool operator>(const Quantity<L, T, A> &lhs, const Quantity<L, T, A> &rhs) {
    return lhs.getValue() > rhs.getValue();
}

template <typename L, typename T, typename A>
constexpr bool operator!=(const Quantity<L, T, A> &lhs, const Quantity<L, T, A> &rhs) {
    return lhs.getValue() != rhs.getValue();
}
