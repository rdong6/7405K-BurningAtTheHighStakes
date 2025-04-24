#pragma once
#include "Spline.h"
#include "lib/geometry/Pose.h"
#include <Eigen/Dense>

class Boomerang final : public ISpline {
public:
	Boomerang(const Pose& start, const Pose& end, double dlead);
	[[nodiscard]] Pose getPoint(double t) const override;
	[[nodiscard]] PoseWithCurvature getPointWithCurvature(double t) const override;

private:
	// contains coefficients for P(t) = a₂t² + a₁t + a₀
	// each row is coefficients for different func
	// row 0 = coefficients of P(t) for X
	// row 1 = coefficients of P(t) for Y
	// row 2 = coefficients of P'(t) for X
	// row 3 = coefficients of P'(t) for Y
	// row 4 = coefficients of P''(t) for X
	// row 5 = coefficients of P''(t) for Y
	//
	// col 0 = a₂
	// col 1 = a₁
	// col 2 = a₀
	Eigen::Matrix<double, 6, 3> coefficients = Eigen::Matrix<double, 6, 3>::Zero();

	// see https://www.desmos.com/calculator/wguveiuycx for the stuff
	// basically characteristic matrix is used to calculate the coefficients for representation as P(t) = a₂t² + a₁t + a₀
	static constexpr Eigen::Matrix3d characteristicMatrix{{+1.0, -2.0, 1.0},
															  {-2.0, +2.0, 0.0},
															  {+1.0, 0.0, 0.0}};
};