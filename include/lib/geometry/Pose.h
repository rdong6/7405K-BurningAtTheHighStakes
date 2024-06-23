#pragma once

class Pose {
private:
	double _x, _y, _theta;

public:
	Pose() = default;
	Pose(double x, double y, double theta = 0) : _x(x), _y(y), _theta(theta){};

	double getX() const;
	double getY() const;
	double getTheta() const;
	double getShortTheta() const;// not implemented!
	double distanceTo(Pose other) const;
	double headingToPoint(Pose other) const;

	double dot(const Pose& other) const;
	Pose lerp(const Pose& other, double t) const;
	Pose transformBy(double delta_x, double delta_y, double delta_theta) const;
	Pose transformBy(Pose other) const;
	static Pose getTransformation(const Pose& A, const Pose& B);
	Pose scale(double scalar) const;
	// Pose exp?????
};

#include "Pose.inl"