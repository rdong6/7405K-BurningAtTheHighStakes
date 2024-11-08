#pragma once
#include "lib/utils/Math.h"
#include <cmath>

// Rotation in 2D Coordinate Frame represented by point on unit circle
// Angle is continous -> doesn't clamp between 0˚-360˚
class Rotation2D {
public:
	constexpr Rotation2D() = default;
	constexpr explicit Rotation2D(double rads) : m_theta(rads), m_cos(std::cos(rads)), m_sin(std::sin(rads)) {}

	// x = cos, y = sin of rotation
	constexpr Rotation2D(double x, double y) {
		// normalize x and y
		double mag = std::hypot(x, y);
		if (mag > 1e-6) {
			m_cos = x / mag;
			m_sin = y / mag;
		} else {
			m_sin = 0;
			m_cos = 1;
		}

		m_theta = std::atan2(m_sin, m_cos);
	}

	constexpr double radians() const { return m_theta; }
	constexpr double degrees() const { return util::toDeg(m_theta); }

	constexpr Rotation2D lerp(const Rotation2D& end, double t) const { return *this + (end - *this) * t; }

	constexpr double cos() const { return m_cos; }
	constexpr double sin() const { return m_sin; }
	constexpr double tan() const { return this->sin() / this->cos(); }

	// adds rotation onto current rotation
	constexpr Rotation2D rotateBy(const Rotation2D& other) const {
		return {cos() * other.cos() - sin() * other.sin(), cos() * other.sin() + sin() * other.cos()};
	}

	constexpr Rotation2D operator+(const Rotation2D& other) const { return rotateBy(other); }
	constexpr Rotation2D operator-(const Rotation2D& other) const { return operator+(-other); }

	// returns inverse of current rotation (negation of current angle)
	constexpr Rotation2D operator-() const { return Rotation2D(-m_theta); }

	// multiplies current rotation by scalar
	constexpr Rotation2D operator*(double scalar) const { return Rotation2D(m_theta * scalar); }
	constexpr Rotation2D operator/(double scalar) const { return operator*(1.0 / scalar); }

	constexpr bool operator==(const Rotation2D& other) const {
		return util::fpEquality(std::hypot(cos() - other.cos(), sin() - other.sin()), 0.0);
	}

private:
	double m_theta = 0;// in radians

	// stores value of trig func's result given current theta
	double m_cos = 1;
	double m_sin = 0;
};