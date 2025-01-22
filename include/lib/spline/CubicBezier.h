#pragma once
#include "Spline.h"
#include "lib/geometry/Pose.h"
#include <Eigen/Dense>

// Representation of 1 cubic bezier curve. Multiple of these can be used to make a spline
class CubicBezier final : public ISpline {
public:
	CubicBezier(const Pose& p0, const Pose& p1, const Pose& p2, const Pose& p3);

	[[nodiscard]] Pose getPoint(double t) const override;
	[[nodiscard]] PoseWithCurvature getPointWithCurvature(double t) const override;

private:
	[[nodiscard]] Pose getFirstDeriv(double t) const;
	[[nodiscard]] Pose getSecondDeriv(double t) const;

	// CLEAN THIS UP -> make it so we can do one multiplication to get everything
	Eigen::Matrix<double, 4, 2> p;           // 4x2 matrix representation of the 4 control points
	Eigen::Matrix<double, 4, 4> coefficients;// characteristic matrix for cubic bezier
	Eigen::Matrix<double, 3, 4> first_deriv_coefficients;
	Eigen::Matrix<double, 2, 4> second_deriv_coefficients;
};