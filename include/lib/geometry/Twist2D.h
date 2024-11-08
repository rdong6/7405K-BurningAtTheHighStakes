#pragma once
#include "lib/utils/Math.h"

// Represents change in dist along a 2D arc since last pose update
// Represents diff between two poses
struct Twist2D {
    double dx;
    double dy;
    double dtheta; // radians

    constexpr bool operator==(const Twist2D& other) const {
        return util::fpEquality(dx, other.dx) && util::fpEquality(dy, other.dy) && util::fpEquality(dtheta, other.dtheta);
    }
    
    // multiple this Twist2D by a scalar
    constexpr Twist2D operator*(double scalar) const {
        return Twist2D{dx * scalar, dy * scalar, dtheta * scalar};
    }
};