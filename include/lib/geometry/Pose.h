#pragma once
#include "Rotation2D.h"
#include "Translation2D.h"
#include "Twist2D.h"
#include "lib/utils/Math.h"

class Transform2D;

class Pose {
public:
	constexpr Pose() = default;
	constexpr Pose(const Translation2D& translation, const Rotation2D& rotation)
	    : m_translation(translation), m_rotation(rotation) {}

	// theta in rads
	constexpr Pose(double x, double y, double theta = 0) : m_translation(x, y), m_rotation(theta) {}

	constexpr Pose(double x, double y, const Rotation2D& rotation) : m_translation(x, y), m_rotation(rotation) {}

	/// GETTER FUNCTIONS

	constexpr double X() const { return m_translation.X(); }
	constexpr double Y() const { return m_translation.Y(); }
	constexpr double theta() const { return m_rotation.radians(); }

	// get access to underlying translation and rotation
	constexpr const Translation2D& translation() const { return m_translation; }
	constexpr const Rotation2D& rotation() const { return m_rotation; }

	constexpr Pose lerp(const Pose& end, double t) const {
		return Pose{translation().lerp(end.translation(), t), rotation().lerp(end.rotation(), t)};
	}

	// what angle to face from current point to face other point
	constexpr Rotation2D headingTo(const Pose& target) const {
		Translation2D deltaTranslation = target.translation() - translation();
		return Rotation2D{std::atan2(deltaTranslation.Y(), deltaTranslation.X())};
	}

	/// TRANSFORMER FUNCTIONS

	// Transforms pose by given transformation (global space)
	// new_translation = curr_translation + transform_translation (rotated into robot frame -> ex: transform's X is 5, robot's
	// heading is 90deg so new_translation's y shifted 5)
	//			-> basically, transform'x x is moving robot's local frame forwards
	// basically transformation is local robot frame, into global cord frame
	// new_rotation = curr_rotation + transform_rotation
	constexpr Pose transformBy(const Transform2D& transform) const;

	// rotates around origin
	constexpr Pose rotateBy(const Rotation2D& rotation) const {
		return Pose{m_translation.rotateBy(rotation), m_rotation.rotateBy(rotation)};
	}

	// returns current pose relative to given pose
	// origin is `other` pose -> returned pose is relative to this origin
	constexpr Pose relativeTo(const Pose& other) const;

	// Obtain a new Pose2d from a (constant curvature) velocity.
	//
	// https://file.tavsys.net/control/controls-engineering-in-frc.pdf section
	// 10.2 "Pose exponential" for a derivation.
	//
	// twist is change in robot'ss coordinate frame. Pose exp returns new pose in global coordinate frame
	constexpr Pose exp(const Twist2D& twist) const;

	// Returns a Twist2d that maps this pose to the end pose. If c is the output
	// of a.Log(b), then a.Exp(c) would yield b.
	//
	// Undoes what exp does. kinda like in math w/ log undoing exponent
	constexpr Twist2D log(const Pose& final) const;

	// apply transformation to pose to get new pose
	constexpr Pose operator+(const Transform2D& transformation) const { return transformBy(transformation); }

	// returns transformation mapping initial pose onto final pose
	constexpr Transform2D operator-(const Pose& initial) const;

	constexpr Pose operator*(double scalar) const { return Pose{m_translation * scalar, m_rotation * scalar}; }
	constexpr Pose operator/(double scalar) const { return operator*(1.0 / scalar); }

	// constexpr bool operator==(const Pose&) const = default;
	constexpr bool operator==(const Pose& other) const {
		return util::fpEquality(X(), other.X()) && util::fpEquality(Y(), other.Y()) && util::fpEquality(theta(), other.theta());
	}


private:
	Translation2D m_translation;
	Rotation2D m_rotation;
};

#include "Transform2D.h"

constexpr Pose Pose::transformBy(const Transform2D& transform) const {
	return Pose{m_translation + transform.translation().rotateBy(m_rotation), m_rotation.rotateBy(transform.rotation())};
}

constexpr Pose Pose::relativeTo(const Pose& other) const {
	const Transform2D transform{other, *this};
	return {transform.translation(), transform.rotation()};
}
constexpr Transform2D Pose::operator-(const Pose& initial) const {
	const Pose pose = this->relativeTo(initial);
	return Transform2D{pose.m_translation, pose.m_rotation};
}

constexpr Pose Pose::exp(const Twist2D& twist) const {
	// https://file.tavsys.net/control/controls-engineering-in-frc.pdf for theorem
	// change in robot's local coordinate frame
	const double dx = twist.dx;        // fwd
	const double dy = twist.dy;        // left
	const double dtheta = twist.dtheta;// ccw rotation

	const double sinTheta = std::sin(dtheta);
	const double cosTheta = std::cos(dtheta);

	// s = sin(∆theta) / ∆theta or it's taylor polynomial's approximation
	// c = (1 - cos(∆theta)) / ∆theta
	double s, c;
	if (util::fpEquality(dtheta, 0.0)) {
		s = 1.0 - 1.0 / 6.0 * dtheta * dtheta;
		c = 0.5 * dtheta;
	} else {
		s = sinTheta / dtheta;
		c = (1 - cosTheta) / dtheta;
	}

	const Transform2D transform{Translation2D{dx * s - dy * c, dx * c + dy * s}, Rotation2D{cosTheta, sinTheta}};

	return *this + transform;// does the final rotation matrix, converting robot cord frame into global cord frame
}

constexpr Twist2D Pose::log(const Pose& final) const {
	const Pose transform = final.relativeTo(*this);
	const double dtheta = transform.theta();
	const double halfDTheta = dtheta / 2.0;

	const double cosMinusOne = transform.rotation().cos() - 1;

	double halfThetaByTanOfHalfDtheta;

	if (std::abs(cosMinusOne) < 1E-9) {
		halfThetaByTanOfHalfDtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
	} else {
		halfThetaByTanOfHalfDtheta = -(halfDTheta * transform.rotation().sin()) / cosMinusOne;
	}

	const Translation2D translationPart = transform.translation().rotateBy({halfThetaByTanOfHalfDtheta, -halfDTheta}) *
	        vsqrtd(halfThetaByTanOfHalfDtheta * halfThetaByTanOfHalfDtheta + halfDTheta * halfDTheta);

	return {translationPart.X(), translationPart.Y(), dtheta};
}


// For PathEditor project -> need to expose stuff for interoperability between C++ & javascript
// custom serialization for Pose -> only exposes it as an (X,Y) pair
// #ifdef PATH_EDITOR
// #include <glaze/glaze.hpp>
// template<> struct glz::meta<Pose> {
// 	using T = Pose;
// 	// JSON -> C++
// 	static constexpr auto read_x = [](Pose& pose, const std::string& input) { pose = Pose(std::stod(input), pose.Y(),
// pose.rotation()); }; 	static constexpr auto read_y = [](Pose& pose, const std::string& input) { pose = Pose(pose.X(),
// std::stod(input), pose.rotation()); };
//
// 	// C++ -> JSON
// 	static constexpr auto write_x = [](const Pose& pose) { return pose.X(); };
// 	static constexpr auto write_y = [](const Pose& pose) { return pose.Y(); };
//
// 	static constexpr auto value = object("x", custom<read_x, write_x>, "y", custom<read_y, write_y>);
// };
// #endif