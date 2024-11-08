#pragma once
#include "Rotation2D.h"
#include "Translation2D.h"

class Pose;

class Transform2D {
public:
	constexpr Transform2D() = default;// transform that maps pose onto itself

	constexpr Transform2D(Translation2D translation, Rotation2D rotation) : m_translation(translation), m_rotation(rotation) {}

	constexpr Transform2D(double x, double y, double rads) : m_translation(x, y), m_rotation(rads) {}

	// makes transform which maps inital pose onto final pose
	constexpr Transform2D(const Pose& initial, const Pose& final);

	constexpr const Translation2D& translation() const { return m_translation; }

	constexpr const Rotation2D& rotation() const { return m_rotation; }

	// Invert the transformation. This is useful for undoing a transformation.
	constexpr Transform2D inverse() const {
		// We are rotating the difference between the translations
		// using a clockwise rotation matrix. This transforms the global
		// delta into a local delta (relative to the initial pose).
		return Transform2D{(-translation()).rotateBy(-rotation()), -rotation()};
	}


	// Composes two transformations. The second transform is applied relative to
	// the orientation of the first.
	constexpr Transform2D operator+(const Transform2D& other) const;

	constexpr Transform2D operator*(double scalar) const { return Transform2D(m_translation * scalar, m_rotation * scalar); }

	constexpr Transform2D operator/(double scalar) const { return operator*(1.0 / scalar); }

	constexpr bool operator==(const Transform2D&) const = default;

private:
	Translation2D m_translation;
	Rotation2D m_rotation;
};


#include "Pose.h"

constexpr Transform2D::Transform2D(const Pose& initial, const Pose& final) {
	// rotate global ∆ (final - initial) by CW rotation matrix to get local ∆

	// We are rotating the difference between the translations
	// using a clockwise rotation matrix. This transforms the global
	// delta into a local delta (relative to the initial pose).
	m_translation = (final.translation() - initial.translation()).rotateBy(-initial.rotation());

	m_rotation = final.rotation() - initial.rotation();
}

constexpr Transform2D Transform2D::operator+(const Transform2D& other) const {
	return Transform2D{Pose{}, Pose{}.transformBy(*this).transformBy(other)};
}