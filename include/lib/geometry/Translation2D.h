#pragma once
#include "lib/geometry/Rotation2D.h"
#include "lib/utils/Math.h"
#include <cmath>

// X, Y component representing a point (X, Y) or a vector [X, Y] on 2D Coordinate System
// Reference Frame (right hand system): robot at origin faces positive X dir, forwards = +x, left = +y
class Translation2D {
public:
	// X, Y components set to 0
	constexpr Translation2D() = default;

	// X, Y componenets set to provided values
	constexpr Translation2D(double x, double y) : m_x(x), m_y(y) {}

	// Basically converts from polar cord to cartesian cord
	constexpr Translation2D(double dist, const Rotation2D& rotation) : m_x(dist * rotation.cos()), m_y(dist * rotation.sin()) {}

	constexpr double X() const { return m_x; }

	constexpr double Y() const { return m_y; }

	constexpr Translation2D lerp(const Translation2D& end, double t) const { return *this + ((end - *this) * t); }

	// take dot product of the two vectors
	// this * other
	constexpr double dot(const Translation2D& other) { return X() * other.X() + Y() * other.Y(); }

	constexpr double distanceTo(const Translation2D& other) const { return std::hypot(other.m_x - m_x, other.m_y - m_y); }

	constexpr Rotation2D angle() const { return Rotation2D{m_x, m_y}; }

	// CCW rotation matrix
	constexpr Translation2D rotateBy(const Rotation2D& rotation) const {
		return {m_x * rotation.cos() - m_y * rotation.sin(), m_x * rotation.sin() + m_y * rotation.cos()};
	}


	constexpr Translation2D operator+(const Translation2D& other) const {
		return Translation2D{m_x + other.m_x, m_y + other.m_y};
	}

	constexpr Translation2D operator-(const Translation2D& other) const { return operator+(-other); }

	// return inverse of current translation (negation of x & y components/rotation by 180˚)
	constexpr Translation2D operator-() const { return Translation2D{-m_x, -m_y}; }

	constexpr Translation2D operator*(double scalar) const { return Translation2D{m_x * scalar, m_y * scalar}; }

	constexpr Translation2D operator/(double scalar) const { return operator*(1.0 / scalar); }

	constexpr bool operator==(const Translation2D& other) const {
		return util::fpEquality(m_x, other.m_x) && util::fpEquality(m_y, other.m_y);
	}

private:
	double m_x = 0;
	double m_y = 0;
};