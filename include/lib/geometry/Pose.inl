#include "lib/geometry/Pose.h"
#include "lib/utils/Math.h"

inline double Pose::getX() const {
	return _x;
}

inline double Pose::getY() const {
	return _y;
}

inline double Pose::getTheta() const {
	return _theta;
}

inline double Pose::getShortTheta() const {
	return 0;
}

inline double Pose::distanceTo(Pose other) const {
	// Distance formula for two points
	return sqrt(pow(_x - other._x, 2.0) + pow(_y - other._y, 2.0));
}

inline double Pose::headingToPoint(Pose other) const {
	// Calculates the angle in radians between two points
	double dx = other._x - _x;
	double dy = other._y - _y;
	// return (M_PI / 2.0) - atan2(dy, dx);
	return std::atan2(dx, dy);
}

inline double Pose::dot(const Pose& other) const {
	return _x * other._x + _y * other._y;
}

inline Pose Pose::lerp(const Pose& other, double t) const {
	return Pose(_x + (other._x - _x) * t, _y + (other._y - _y) * t, _theta);
}

inline Pose Pose::transformBy(double delta_x, double delta_y, double delta_theta) const {
	// Transforms the position by certain values
	return {_x + delta_x, _y + delta_y, _theta + delta_theta};
}

inline Pose Pose::transformBy(Pose other) const {
	// Transforms the position by another position
	return transformBy(other._x, other._y, other._theta);
}

inline Pose Pose::getTransformation(const Pose& A, const Pose& B) {
	// Gets the difference in positions between two positions
	return {B._x - A._x, B._y - A._y, B._theta - A._theta};// no return function??
}

inline Pose Pose::scale(double scalar) const {
	// Scales the position by a certain amount
	return {_x * scalar, _y * scalar, _theta};
}